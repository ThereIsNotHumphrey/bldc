# FOC / 电机控制核心地图

本文件只列 **本树里真实存在** 的文件和符号。Clarke/Park 没有独立函数名，位置在 ISR 内联代码。

## 文件职责

| 路径 | 角色 | 移植时 |
| --- | --- | --- |
| `motor/foc_math.c` / `foc_math.h` | 观测器、PLL、SVPWM、速度/位置 PID、霍尔/编码器角度融合、弱磁、HFI 角度校正、`foc_precalc_values()` | 算法主体，优先保留 |
| `motor/mcpwm_foc.c` / `mcpwm_foc.h` | FOC 状态机、ISR、电流环、HFI 注入、开环线程、检测例程、**以及** F4 定时器/ADC/DMA | 必须拆：算法留下，`timer_reinit` / `start_pwm_hw` / DMA 重写 |
| `motor/mc_interface.c` / `mc_interface.h` | 对上 API：设电流/速度/开环、故障、双电机线程选择 | 接口可留，内部的故障 GPIO / 采样缓冲按新板裁剪 |
| `motor/mcpwm.c` / `mcpwm.h` | 六步 BLDC/DC，`commutate()` | v1 FOC 移植不需要 |
| `motor/mcconf_default.h` | `MCCONF_FOC_*` 默认值 | 作参数表，不是驱动 |
| `motor/virtual_motor.c` | αβ 电压驱动的仿真电机 | 可选，用来离线看观测器 |
| `datatypes.h` | `mc_foc_sensor_mode`、`mc_foc_observer_type`、`mc_control_mode`、`mc_configuration` | 只抽取 FOC 字段 |
| `util/utils_math.c` | `utils_fast_sincos_better()`、角度归一化、`utils_fast_atan2` | 观测器和 SVPWM 依赖它 |
| `conf_general.c` | 电阻/电感/磁链/传感器检测与 Kp/Ki/观测器增益换算 | 辨识可后做；公式在 `conf_general_calc_apply_foc_cc_kp_ki_gain()` |
| `encoder/encoder.c` | `encoder_init()` 按 `m_sensor_port_mode` 分派 | v1 若用霍尔或纯开环，可只留 ABI 或整目录后移 |
| `encoder/enc_*.c` | ABI、AS504x、AD2S1205、BiSS-C、MT6816、MA782、PWM、sin/cos 等 | 每种都绑了具体 SPI/TIM 脚 |

## 变换与 PWM

### Clarke（`mcpwm_foc_adc_int_handler`，约 3298–3306 行）

运行且 `MC_STATE_RUNNING` 时：

- 三电阻、`full_clarke == true`：  
  `i_alpha = (2/3) ia - (1/3) ib - (1/3) ic`  
  `i_beta = ONE_BY_SQRT3 * ib - ONE_BY_SQRT3 * ic`
- 否则（两电阻，或三电阻里丢掉饱和的一相后再用平衡假设）：  
  `i_alpha = ia`  
  `i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib`  
  且 `curr2 = -(curr0 + curr1)`（约 3078–3081 行）

`full_clarke` 由 `HW_HAS_3_SHUNTS` 和 `foc_current_sample_mode` 决定。模式枚举 `mc_foc_current_sample_mode`：`FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO`、`ALL_SENSORS`、`HIGH_CURRENT`。`BEST_SENSOR` 在枚举注释里写明 experimental, not used，对应分支包在 `#if 0` 里。

### Park 与反 Park（`control_current`）

```text
id =  c * i_alpha + s * i_beta
iq =  c * i_beta  - s * i_alpha
mod_alpha_raw = c * mod_d - s * mod_q
mod_beta_raw  = c * mod_q + s * mod_d
```

`c`、`s` 来自 `utils_fast_sincos_better(state.phase, ...)`，在选完角度之后、进 `control_current()` 之前计算。

电压归一化写在函数注释里：`voltage_normalize = 1.5 / v_bus`，使 `mod_* == 1` 对应最大相电压矢量（含过调制）。未过调制的限幅是 `max_v_mag = ONE_BY_SQRT3 * max_duty * v_bus * foc_overmod_factor`。

### SVPWM：`foc_svm()`

`motor/foc_math.c`。注释要求幅值不超过 `sqrt(3)/2`（0.866）以免过调制，并指向仓库 PR 372 的说明。扇区由 αβ 象限判定，输出 `tA/tB/tC` 为 0…`PWMFullDutyCycle` 的比较值，并回写 `svm_sector`。调用点：

- `control_current()` 末尾，每个控制周期一次，接着 `TIMER_UPDATE_DUTY_M1`
- HFI 且不是 `FOC_CONTROL_SAMPLE_MODE_V0_V7` 时，把 **下一拍** 的 HFI 矢量先变成 `m_duty1_next` 等，由下一次 ISR 开头写入
- `FOC_CONTROL_SAMPLE_MODE_V0_V7_INTERPOL` 时，在对面零矢量用半拍相位插值再调一次 `foc_svm()`

`foc_control_sample_mode`：`FOC_CONTROL_SAMPLE_MODE_V0`、`V0_V7`、`V0_V7_INTERPOL`。没有相分流宏 `HW_HAS_PHASE_SHUNTS` 时，V7 半周 ISR 直接 return（约 2952–2954 行），控制频率是开关频率的一半。`foc_precalc_values()` 里 `p_fs = foc_f_zv * 0.5` 与此一致；有相分流且模式为 `V0_V7` 时 `p_fs = foc_f_zv`。

## 电流环：`control_current()`

静态函数，仅被 `mcpwm_foc_adc_int_handler()` 在 `MC_STATE_RUNNING` 时调用。文件内注释列出了调用前必须设好的量：`id_target`、`iq_target`、`max_duty`、`phase`、`i_alpha`、`i_beta`、`v_bus`。

顺序：

1. 按速度决定是否 `do_hfi`（见 startup 文档）。HFI 未完成前把 `iq_target` 置 0。
2. Park，得到 `id`/`iq`。低通 `id_filter`/`iq_filter` 用 `foc_current_filter_const`，**不进** PI 反馈。
3. 误差 `Ierr_d/q = target - measured`。积分 `vd_int/vq_int += Ierr * ki * dt`，`ki` 为 `foc_current_ki`，温度补偿打开时用 `m_current_ki_temp_comp`。
4. `vd/vq = 积分 + Ierr * foc_current_kp`。注释写明对象是一阶 `1/(Ls+R)`，没有 D 项。
5. 解耦 `mc_foc_cc_decoupling_mode`：`DISABLED`、`CROSS`（`ω*iq*Lq` / `ω*id*Ld`）、`BEMF`（`ω*ψ`）、`CROSS_BEMF`。`ω` 用 `m_speed_est_fast`。`Lq/Ld` 来自 `foc_precalc_values()`：`p_lq = L + ld_lq_diff/2`，`p_ld = L - ld_lq_diff/2`，`L` 是 `foc_motor_l`。
6. d 轴优先限幅，q 轴用剩余电压圆。
7. 反 Park，`update_valpha_vbeta()` 把调制或相电压采样变回 `v_alpha/v_beta`（观测器的电压输入）。
8. 可选音频调制（`m_audio`），沿 q 轴加电压。v1 可删。
9. HFI 注入，再 `foc_svm()`，写 CCR。
10. 三相占空相等且 `foc_short_ls_on_zero_duty` 时改走 `full_brake_hw()`，而不是调制一个零矢量。

目标电流在进本函数之前由 ISR 算好：占空模式把 `m_duty_cycle_set` 变成 `iq_set_tmp`（带 `foc_duty_dowmramp_kp/ki`），制动模式按速度符号取反，然后 MTPA（`foc_mtpa_mode`，需要 `foc_motor_ld_lq_diff != 0`）、`foc_run_fw()` 弱磁、母线/相电流限幅 `lo_current_*` 与 `lo_in_current_*`。

## 速度环与“转矩”

| 函数 | 线程 | 输入 | 输出 |
| --- | --- | --- | --- |
| `foc_run_pid_control_speed()` | `"foc pid"`，`mcpwm_foc.c` 的 `pid_thread` | `m_speed_pid_set_rpm` 与估计转速 | 写入电流设定（q 轴） |
| `foc_run_pid_control_pos()` | 同上 | `m_pos_pid_set` 与 `m_pos_pid_now` | 速度或电流，见函数内部 |
| `mc_interface_set_current()` | 任意命令线程 | 安培 | `mcpwm_foc_set_current()` → `m_iq_set`，`m_control_mode = CONTROL_MODE_CURRENT` |
| `mc_interface_set_pid_speed()` | 同上 | ERPM | `CONTROL_MODE_SPEED` |

PID 周期枚举在配置的 `sp_pid_loop_rate`（25 Hz … 10 kHz），**慢于**电流环。位置反馈优先用 `encoder_read_deg()`；没有编码器时用 `RAD2DEG(state.phase)`，并按 `p_pid_ang_div` 做减速比展开（ISR 末尾约 3880–3912 行）。

v1 可以只实现 `CONTROL_MODE_CURRENT`：转矩指令 = `iq_target`。速度环是外环，不挡电流环打通。

## 观测器

`foc_observer_update()` 开头注释指向 Ortega / Praly 论文。输入 `v_alpha, v_beta, i_alpha, i_beta, dt`，状态 `observer_state.x1/x2`，输出电角度 `atan2(x2 - L*i_beta, x1 - L*i_alpha)`。

`mc_foc_observer_type`：

| 枚举 | 行为要点 |
| --- | --- |
| `FOC_OBSERVER_ORTEGA_ORIGINAL` | 非线性磁链观测器，误差为正时被夹到 0 |
| `FOC_OBSERVER_ORTEGA_LAMBDA_COMP` | 同上，并自适应 `lambda_est` |
| `FOC_OBSERVER_MXLEMMING` | 电压积分减 `L*Δi`，幅值夹到 `λ`。注释标明作者 David Molony / MESC |
| `FOC_OBSERVER_MXLEMMING_LAMBDA_COMP` | 同上 + `lambda_est` |
| `FOC_OBSERVER_MXV` | 纯电压积分，幅值超 `λ` 则缩放 |
| `FOC_OBSERVER_MXV_LAMBDA_COMP` | 带 `lambda_est` 的幅值限制 |
| `FOC_OBSERVER_MXV_LAMBDA_COMP_LIN` | `lambda_est` 用低通跟踪幅值 |

默认（`motor/mcconf_default.h`）：`MCCONF_FOC_OBSERVER_TYPE` = `FOC_OBSERVER_MXLEMMING_LAMBDA_COMP`。

增益 `m_gamma_now` 不在 ISR 里从零算，而在 1 ms 的 `timer_update()`：按占空从 0 映射到 `foc_observer_gain`，下限 `foc_observer_gain_slow * foc_observer_gain`，再乘 4。电感随 `foc_motor_ld_lq_diff` 和 `id/iq` 方向修正；饱和补偿看 `foc_sat_comp_mode`。电阻在 `foc_temp_comp` 时用 `m_res_temp_comp`。

ISR 在观测器角度上再加 `m_pll_speed * dt * (0.5 + foc_observer_offset)`，补偿 PWM 滞后。`foc_observer_offset` 默认 `-1`（单位是控制周期）。

速度有两路：`foc_pll_run()` → `m_pll_phase` / `m_pll_speed`（增益 `foc_pll_kp`、`foc_pll_ki`）；以及相邻拍角度差 × `p_fs` 的 `m_speed_est_fast`。相位源由 `foc_speed_soure` 选 `FOC_SPEED_SRC_CORRECTED`（融合后的 `state.phase`）或 `FOC_SPEED_SRC_OBSERVER`。

`timer_update()` 里还有一个在线电阻观测器（注释引用 adaptive flux observer 论文），结果在 `m_res_est`，**不**自动覆盖 `foc_motor_r`。读接口是 `mcpwm_foc_get_est_res()` / `get_est_lambda()` / `get_est_ind()`。

## HFI

状态 `hfi_state_t`（`foc_math.h`）：缓冲、`angle`、`double_integrator`、`est_done_cnt`、`flip_cnt`、`observer_zero_time`。

| 符号 | 位置 | 作用 |
| --- | --- | --- |
| `control_current()` 后半段 `if (do_hfi)` | `mcpwm_foc.c` | 在 αβ 调制上叠加 `foc_hfi_voltage_*`，并采样 `di` |
| `hfi_update()` | 同文件，线程 `"foc hfi"`，`chThdSleepMicroseconds(500)` | 对经典旋转注入做 bin0/bin1/bin2 FFT，解 180° 模糊 |
| `foc_hfi_adjust_angle()` | `foc_math.c` | V2–V5 在低速用电流差修正 `m_hfi.angle` |
| `update_hfi_samples()` | `mcpwm_foc.c` | 按 `foc_hfi_samples` 绑定 8/16/32 点 FFT 函数指针 |
| `utils_tab_sin_32_1` / `utils_tab_cos_32_1` | `util/utils_math.c` | 经典 HFI 的旋转注入表 |

传感器枚举见 startup 文档。电感检测会 **临时** 把 `foc_sensor_mode` 设成 `FOC_SENSOR_MODE_HFI`，那是测量，不是运行时角度源。

## 开环角度源（指令，不是传感器模式）

| API | 模式 | ISR 行为 |
| --- | --- | --- |
| `mcpwm_foc_set_openloop_current(current, rpm)` | `CONTROL_MODE_OPENLOOP` | `m_openloop_angle += dt * m_openloop_speed`，`phase` 被覆盖；电流走 q 轴 |
| `mcpwm_foc_set_openloop_phase(current, phase)` | `CONTROL_MODE_OPENLOOP_PHASE` | `phase = m_openloop_phase`（度转弧度） |
| `mcpwm_foc_set_openloop_duty` / `_duty_phase` | `CONTROL_MODE_OPENLOOP_DUTY` / `OPENLOOP_DUTY_PHASE` | 同上，但外环按占空限电流 |
| `mcpwm_foc_set_handbrake` | `CONTROL_MODE_HANDBRAKE` | `phase` 强制为 0，电流把转子锁在固定电角度 |
| `m_phase_override` + `m_phase_now_override` | 检测函数内部 | 电阻、霍尔表、编码器偏移检测时锁死或扫相 |

无感 **运行时** 的开环斜坡是另一套变量：`m_phase_now_observer_override`、`m_phase_observer_override`、`m_min_rpm_timer`，由 `timer_update()` 维护。不要和 `CONTROL_MODE_OPENLOOP` 混成一个模式。

## 编码器路径

`encoder_init()`（`encoder/encoder.c`）看 `mc_configuration.m_sensor_port_mode`（`SENSOR_PORT_MODE_HALL`、`ABI`、`AS5047_SPI`、`AD2S1205`、`SINCOS`、`MT6816_SPI_HW`、`BISSC`、`TLE5012_SSC_SW` 等）。FOC 是否使用这只角度，还要看 `foc_sensor_mode` 是不是 `FOC_SENSOR_MODE_ENCODER` 或 `FOC_SENSOR_MODE_ENCODER_AB`。

ISR 里：

- `encoder_read_deg()` → 乘 `foc_encoder_ratio`，减 `foc_encoder_offset`，可选 `foc_encoder_inverted` 和 `g_backup.enc_corr[]` → `m_phase_now_encoder`
- `foc_correct_encoder(obs, enc, speed, foc_sl_erpm)`：转速高于 `foc_sl_erpm`（带 5% 滞环）改用观测器，低于则用编码器角。HFI 模式复用这个函数，只是把“编码器角”换成 `m_hfi.angle`，阈值换成 `foc_sl_erpm_hfi`
- 有索引的 `FOC_SENSOR_MODE_ENCODER`：`encoder_index_found()` 为假时，`phase = m_phase_now_encoder_no_index`（开环积分，用来找索引）
- `FOC_SENSOR_MODE_ENCODER_AB`：无索引。`|speed| >= foc_sl_erpm` 时 `encoder_set_deg()` 把编码器对齐到观测器；在此之前若开环覆盖有效，用覆盖角，否则用观测器

检测：`mcpwm_foc_encoder_detect()` 用 d 轴电流扫 `m_phase_now_override`，求 `offset/ratio/inverted`。`conf_general_autodetect_apply_sensors_foc()` 在霍尔失败后还会试 AS5047（`SENSOR_PORT_MODE_AS5047_SPI` + `mc_interface_set_openloop_phase`）。

## 霍尔路径

运行时只有一条：`foc_correct_hall(observer_angle, dt, motor, utils_read_hall(...))`。

- 霍尔码 0…7 查 `foc_hall_table[]`。值 `< 201` 才有效（默认 `MCCONF_FOC_HALL_TAB_*` 是 255，即未标定）。
- 表值 / 200 * 2π 得到扇区角。高于 `foc_hall_interp_erpm` 时在 60° 扇区内插；低于则只用扇区中心，避免换向时粘在 60° 误差上。
- `m_using_hall`：`|m_pll_speed| < foc_sl_erpm` 时输出霍尔角，否则输出观测器角。
- 读脚：`utils_read_hall()` → `utils_read_hall_hw()`（`util/utils_sys.c`）对 `READ_HALL1/2/3()` 做多数表决。这些宏在板级头文件。

标定：`mcpwm_foc_hall_detect()`。d 轴电流锁定后，`m_phase_now_override` 正反转各三圈，把每个霍尔码上的 sin/cos 平均成表。`conf_general_autodetect_apply_sensors_foc()` 成功时把 `foc_sensor_mode` 设为 `FOC_SENSOR_MODE_HALL` 并写入 `foc_hall_table`。

旧 BLDC 的霍尔表是另一套：`mcpwm_init_hall_table()` / `mcpwm_get_hall_detect_result()`，服务 `SENSOR_MODE_SENSORED`，与 `foc_hall_table` 不要混用。

## 弱磁、MTPA、死区补偿电压

- 弱磁：`foc_run_fw()`。当调制接近满时增加 `m_i_fw_set`，ISR 把它加到负 d 轴，并按 `foc_fw_q_current_factor` 削减 q 轴。`mcpwm_foc_set_fw_override()` 可强制。
- MTPA：ISR 内公式 `id = (λ - sqrt(λ² + 8 (Ld-Lq)² iq²)) / (4 (Ld-Lq))`，注释指向 bldc PR 179。
- 相电压重建：`update_valpha_vbeta()`。有采样时用 `ADC_V_L*_VOLTS` 减 `foc_offsets_voltage`；否则由调制反推。死区相关量在配置 `foc_dt_us`（默认 0.12 µs），具体补偿在 `update_valpha_vbeta()` 后半段。观测器吃的是补偿后的 `v_alpha/v_beta`，电流 PI 的输出占空不经过这条补偿改写（`control_current` 注释写明 dead-time compensated vd/vq 不用于开关时刻）。

## 梯形波路径（对照，不是 FOC v1）

`motor/mcpwm.c`：`set_next_comm_step()`、`commutate()`、`update_sensor_mode()`。传感器枚举是 `mc_sensor_mode`（`SENSOR_MODE_SENSORLESS` / `SENSORED` / `HYBRID`），换相方式是 `mc_comm_mode`（`COMM_MODE_INTEGRATE` / `COMM_MODE_DELAY`）。入口 `mc_interface_init()` 在 `motor_type != MOTOR_TYPE_FOC` 时调用 `mcpwm_init()`。FOC 的 `foc_sensor_mode` 与这里的 `sensor_mode` 是两套配置字段。
