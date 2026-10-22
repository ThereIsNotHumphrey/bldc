# 参数辨识与转子角

## 直接结论

**Rs、Ld、Lq、ψf 的辨识不产生转子电角度。** 这些量是观测器和电流 PI 的系数。电角度 θ 在运行时来自开环积分、霍尔表、编码器或 HFI 的凸极/饱和方向。辨识过程里电机常常被锁在一个 **人为指定** 的相位上，那个相位是测试条件，不是测量结果，也不会作为无感启动角写进 `mc_configuration`。

本树没有 IPD 函数。和“上电知道转子在哪”最接近的运行时机制是 HFI 的模糊分辨（`foc_hfi_amb_mode`），它不在 `conf_general_detect_apply_all_foc()` 的写回字段里。

## “检测电机”实际写回什么

`conf_general_detect_apply_all_foc()`（`conf_general.c`）成功时改的是 **原来的** `mcconf_old`：

| 写回字段 | 来源 |
| --- | --- |
| `l_current_max` / `l_current_min` | `measure_r_l_imax()` 用 `sqrt(max_power_loss / R / 1.5)`，再夹到 `HW_LIM_CURRENT` |
| `l_abs_current_max` | 约 1.5 倍，受 `HW_LIM_CURRENT_ABS` 限制 |
| `motor_type` | 固定 `MOTOR_TYPE_FOC` |
| `foc_motor_r` | 电阻 |
| `foc_motor_l` | 平均电感（H，检测函数内部先以 µH 返回再 `* 1e-6`） |
| `foc_motor_ld_lq_diff` | Lq − Ld |
| `foc_motor_flux_linkage` | 开环拖转得到的 ψ |
| `foc_encoder_offset` / `foc_encoder_ratio` / `foc_encoder_inverted` | **仅当** 开环磁链测量期间编码器在转且算出非负 offset/ratio |
| `foc_temp_comp`、`foc_temp_comp_base_temp` | 电机温度传感器高于 −10 °C 时 |
| `foc_current_kp`、`foc_current_ki`、`foc_observer_gain` | `conf_general_calc_apply_foc_cc_kp_ki_gain(mcconf, 1000)` |

没有写入 `m_phase_now_*`、霍尔表或 HFI 角。测量开始前强制 `foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS`，结束后保留调用前的传感器模式（写的是 `mcconf_old`，而临时改模式的是副本 `mcconf`）。

增益公式（时间常数 `tc` 单位 µs，这里传入 1000）：

```text
bw = 1 / (tc * 1e-6)
foc_current_kp = L * bw
foc_current_ki = R * bw
foc_observer_gain = (1e-3 / λ²) * 1e6
```

λ 或 L 为 0 时这个换算没有意义，所以顺序必须是先 R/L，再 ψ，再增益。

终端入口：`foc_detect_apply_all`（`terminal.c`）。分步命令对应 `foc_measure_r`、`foc_measure_l`、`foc_measure_flux` 一类处理，最终调用下面三个测量函数。协议枚举是 `COMM_DETECT_MOTOR_R_L`、`COMM_DETECT_MOTOR_FLUX_LINKAGE`、`COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP`、`COMM_DETECT_APPLY_ALL_FOC`。

## 电阻：`mcpwm_foc_measure_resistance()`

做法：

1. `m_phase_override = true`，`m_phase_now_override = 0`。电角度被 **指定** 为 0，不是估出来的。
2. `m_id_set = 0`，把 `m_iq_set` 在约 200 ms 内斜坡到目标电流（每毫秒一步 `current/200`）。固定相位下的 q 轴电流在定子坐标里是一个静止矢量，转子会被吸到与该矢量对齐的位置。注释没有把这一步叫成角度测量；它只是让电流进入稳态。
3. 等 50 ms，然后等 `timer_update()` 里累加的 `m_samples` 达到 `samples`。那里把 `sqrt(vd²+vq²)` 和 `sqrt(id²+iq²)` 相加。
4. `*resistance = voltage_avg / current_avg`。
5. `stop_after` 为真时清电流、清除相位覆盖、`stop_pwm_hw()`。

量到的是 **V/I**。相位覆盖在函数返回前被清掉（`stop_after == true` 时），配置里不出现 θ。

`measure_r_l_imax()` 会从小电流往上试，直到 `I²R * 1.5` 接近 `max_power_loss/5`，再用该电流采 100 点。这只限制发热，不改善角度信息。

## 电感与 Ld/Lq：`mcpwm_foc_measure_inductance()`

做法与“转起来测电流”不同，是短时电压注入：

1. 暂时保存并改写配置：`foc_sensor_mode = FOC_SENSOR_MODE_HFI`，`foc_hfi_amb_mode = FOC_AMB_MODE_SIX_VECTOR`，`foc_hfi_samples = HFI_SAMPLES_32`，`foc_control_sample_mode = FOC_CONTROL_SAMPLE_MODE_V0`，`foc_current_sample_mode = FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO`，HFI 电压由参数 `duty` 换算，`foc_sl_erpm_hfi = 20000`（注入在很低速以外都保持），`foc_f_zv` 若高于 30 kHz 则降到 30 kHz。
2. `mcpwm_foc_set_duty(0)` 让状态机运行，从而在 `control_current()` 里做旋转注入，在 `hfi_update()` 里填缓冲。
3. 读 FFT：`fft_bin0_func` 的实部是电感 **倒数** 的平均；`fft_bin2_func` 是二次谐波幅值。然后

```text
Ld_est = 1 / (offset + amplitude)
Lq_est = 1 / (offset - amplitude)
L      = (Ld + Lq) / 2
ld_lq_diff = Lq - Ld
```

4. 结果乘 `1e6` 得到 µH，再乘 `ind_scale_factor = 0.9`。注释说明曾经是 0.8，为了让观测器侧电感偏小而不是偏大，并补偿延迟造成的高估。
5. 配置恢复为测量前的值。

`hfi_update()` 在同一缓冲上 **会** 算出 `angle_bin_2`，但 `mcpwm_foc_measure_inductance()` 没有读取或保存它。二次谐波的相位正是凸极相对定子的方向，本函数把它丢掉，只留幅值。因此 **Ld/Lq 检测 ≠ 初始角**。

`mcpwm_foc_measure_inductance_current()` 只是把 `duty` 从 0.02 往上乘 1.5，直到电流样本达到 `curr_goal`，再调用上面的函数。

`mcpwm_foc_measure_res_ind()` 是电阻 + 电感的组合封装，供终端和 `COMM_DETECT_MOTOR_R_L` 使用，返回值同样没有角度。

## 磁链：`conf_general_measure_flux_linkage_openloop()`

前提：`res > 0` 且 `ind > 0`，否则直接返回。它自己用 `tc = 1500` µs 算一套临时 Kp/Ki，设 `foc_sensor_mode = SENSORLESS`，`foc_cc_decoupling = DISABLED`，然后：

1. `mc_interface_set_openloop_current()`：200 ms 内电流斜坡、转速仍为 0，把转子锁在开环相位的起点。锁的是力学位置，函数不记录该角度。
2. 采 1 s 静止占空 `duty_still`。
3. 每个毫秒把开环 ERPM 增加 `erpm_per_sec/1000`，直到 `|duty|` 达到目标、超时、或转速到 12000 ERPM。`detect_apply_all_foc` 传入的是电流 `i_max/2.5`、占空 0.3、加速度 1800 ERPM/s。
4. 稳定后采 10000 次 `vd/vq/id/iq`：

```text
v_mag = hypot(vq_avg, vd_avg)
i_mag = hypot(iq_avg, id_avg)
rad_s = RPM2RADPS(rpm_now)          // 开环指令转速，不是观测器转速
linkage = (v_mag - res * i_mag) / rad_s - i_mag * ind
```

这是稳态相量关系 `V ≈ R I + jω (L I + ψ)` 的幅值形式，解出的是 **ψ（Wb）**。`rpm_now` 是函数自己积分的开环指令，所以这个 ψ 依赖于“电机确实跟着开环转速在转”的假设；代码用占空塌陷（`duty_now < duty_max * 0.7`）等条件把失败标成 `linkage = -1/-2/-3`。

5. 若驱动段成功，把临时 λ 和 `foc_observer_gain = 0.5e3 / λ²` 写进 **临时** 配置，停 PWM，再在惯性滑行中累加 `mcpwm_foc_get_vq() / ω` 得到 `linkage_undriven`。`detect_apply_all_foc` 在无驱动样本数 `> 60` 时改用这个无驱动值。

滑行循环里如果编码器在动，会顺便估计 `enc_ratio`、`enc_inverted`、`enc_offset`（编码器角度与 `mcpwm_foc_get_phase_bemf()` 的差的圆平均）。失败时这些输出被设成 `-1` / `false`。`detect_apply_all_foc` 只有在 offset 和 ratio **都 ≥ 0** 时才写入电机配置。没有编码器时它们保持 `-1`，不会伪造一个转子零位。

同文件还有 `conf_general_measure_flux_linkage()`（带 `current, duty, min_rpm` 的另一套，约 742 行），同样解磁链，不返回 θ。

## 真正会得到角度的检测（另一组函数）

这些 **不是** R/L/ψ 检测的一部分：

| 函数 | 得到什么 | 怎么得到 |
| --- | --- | --- |
| `mcpwm_foc_hall_detect()` | `foc_hall_table[8]` | d 轴电流，开环扫相，记录霍尔码对应的电角度 |
| `mcpwm_foc_encoder_detect()` | 编码器 `offset`、`ratio`、`inverted` | d 轴电流扫相，与 `m_phase_now_encoder` 比较；先尝试转到 `encoder_index_found()` |
| `conf_general_autodetect_apply_sensors_foc()` | 把模式设为霍尔、AS5047 或无感 | 先霍尔检测；失败再开环扫 180° 看 AS5047 是否跟随；都不成则 `FOC_SENSOR_MODE_SENSORLESS` |
| 运行时 `hfi_update()` | `m_hfi.angle`，可选地翻转 π | 凸极 FFT 或 d 轴饱和脉冲。只活在 RAM 的 `hfi_state_t`，检测电机流程不保存它 |

`conf_general_autodetect_apply_sensors_foc()` 的返回约定写在注释里：`2` AS5147，`1` 霍尔，`0` 无感，`-1` 失败。终端命令是 `foc_sensors_detect_apply`。

## 和启动的关系（避免把参数当成位置）

无感开环斜坡（`timer_update()`）使用已辨识的 `foc_motor_flux_linkage` 去 **初始化观测器状态的长度**，使用 `foc_motor_r` / `foc_motor_l` 去跑观测器方程。它不读取辨识过程中转子被吸住的那个角度——那个角度在测量函数返回时已经随 `m_phase_override = false` 丢掉，而且电阻测量把相位定在 0，只说明“电流矢量在定子 β 轴附近”，不能在下次上电复现转子位置（转子可以在断电后被转走）。

因此教学上应把两件事分开：

1. **参数 ID**：`detect_apply_all_foc` → R、L、Ld−Lq、ψ、PI 和观测器增益。
2. **每个上电时刻的 θ**：开环拖转、霍尔、编码器或 HFI ambiguity。HFI 的六矢量/双脉冲可以在 **当次** 上电得到 θ，但那是 `foc_sensor_mode` 的运行时行为，不是参数 ID 的输出。
