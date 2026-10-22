# 学习路线（只学 FOC 骨架）

面向要把 **电流环、SVPWM、一种角度** 移植走的人。不覆盖 LispBM、QML、CAN 协议细节、BMS、IMU 和全部编码器芯片。每一步都先读列出的符号，再往下。

## 0. 先建立坐标系

读 `datatypes.h` 里这几个枚举，不要通读整个 `mc_configuration`：

- `mc_motor_type`：FOC 与六步是两条固件。
- `mc_foc_sensor_mode`：运行时 θ 的开关。
- `mc_control_mode`：电流、占空、速度、开环。
- `mc_foc_observer_type`、`mc_foc_cc_decoupling_mode`、`mc_foc_hfi_amb_mode`。

然后看 `motor/foc_math.h` 的 `motor_state_t` 与 `motor_all_state_t`。后面所有函数都在改这块内存。

## 1. 一次 PWM 周期在做什么

只读 `mcpwm_foc_adc_int_handler()` 中 `MC_STATE_RUNNING` 分支，配 `01-architecture-overview.md` 的流程图。故意跳过函数开头的双电机和 V7。记下四个副作用：`i_alpha/i_beta`、`phase`、`id_target/iq_target`、`control_current()` 写出的 CCR。

对照 `main.c` 顶部注释，确认 FOC 的快环 **不是** `ADC1_2_3_IRQHandler`。那个 IRQ 在 `mc_interface_adc_inj_int_handler()` 里对 FOC 是空的。

## 2. SVPWM 与电流 PI（可以还没有角度理论）

1. `foc_svm()`：扇区、T1/T2、三相比较值如何相加到 `PWMFullDutyCycle/2`。
2. `control_current()` 从 Park 到反 Park，先忽略 `do_hfi` 整段和音频 `switch`。
3. 看电压圆限幅为什么 d 轴优先。
4. 看 `TIMER_UPDATE_DUTY_M1`：比较值最后写到哪里，以及 `HW_HAS_3_SHUNTS` 为何交换 CCR2/CCR3。

练习：在纸上令 `phase = 0`、`vd = 0`、`vq > 0`，写出 `mod_alpha/mod_beta`，再看 `foc_svm` 哪一相最大。这就是 `OPENLOOP_PHASE` 锁转子时的电压方向。

## 3. 角度从哪来（仍然不要先读 HFI）

读 ISR 里 `switch (foc_sensor_mode)` 和紧跟的开环/手刹/`m_phase_override`。对照 `03-startup-and-sensorless.md`。

然后只读 `timer_update()` 从 `foc_sl_openloop_time_lock` 到 `m_phase_observer_override = false` 的那一段。问题：为什么无感低速不相信 `foc_observer_update()` 的 `atan2`？代码的答案是滞环时间到了就覆盖状态，并预置 `x1/x2`，而不是去测初始角。

霍尔只读 `foc_correct_hall()`。编码器只读 `foc_correct_encoder()` 和 ISR 里把 `encoder_read_deg()` 收成 `m_phase_now_encoder` 的几行。

## 4. 观测器（无感的高速部分）

`foc_observer_update()` 按 `foc_observer_type` 分叉。第一遍只读默认的 `FOC_OBSERVER_MXLEMMING` / `MXLEMMING_LAMBDA_COMP` 和最后的 `atan2`。输入电压从 `update_valpha_vbeta()` 来，电流来自 Clarke。

增益不在这个函数里调，在 `timer_update()` 的 `m_gamma_now`。`conf_general_calc_apply_foc_cc_kp_ki_gain()` 给出一组和 λ、L、R 绑定的数量级。

`foc_pll_run()` 单独看：它不产生 θ 给电流环（电流环用的是融合后的 `phase`），它产生 `m_pll_speed`。低延迟速度是相邻 `phase` 差分。

## 5. 参数是什么、不是什么

读 `04-parameter-id-vs-rotor-angle.md`，再对着这三个函数跳读，不要通读 `conf_general.c`：

- `mcpwm_foc_measure_resistance()`：相位被写成 0，结果是 V/I。
- `mcpwm_foc_measure_inductance()`：临时 HFI，结果是 L 和 Ld−Lq，不保存 `angle_bin_2`。
- `conf_general_measure_flux_linkage_openloop()`：开环加速，公式 `(V - RI)/ω - LI`。

然后看 `conf_general_detect_apply_all_foc()` 写回 `mcconf_old` 的那一段字段列表，确认没有 θ。

## 6. HFI（可选，放在电流环移植之后）

顺序：

1. `control_current()` 里 `do_hfi` 的条件，以及未完成时 `iq_target = 0`。
2. 经典模式如何用 `utils_tab_sin_32_1` 叠加电压并记录 `di`。
3. `hfi_update()` 的 bin2 角度与 `flip_cnt`。
4. `FOC_AMB_MODE_D_SINGLE_PULSE` / `D_DOUBLE_PULSE` 如何改 `id_target` 并 ±π。
5. `FOC_SENSOR_MODE_HFI_START` 如何把该角度写进 `observer.x1/x2` 后停止注入。

这一步对应“静止获得 θ”。它使用凸极和饱和，使用的 **前提** 仍是已经知道 L 的量级（`p_inv_ld_lq` 来自 `foc_precalc_values()`），但函数返回值不是 L。

## 7. 速度环，作为外环

`pid_thread` → `foc_run_pid_control_speed()`。确认它写的是电流设定，并且周期由 `sp_pid_loop_rate` 决定，比 ADC 回调慢。位置环 `foc_run_pid_control_pos()` 依赖 `m_pos_pid_now`，而该角度在 ISR 末尾由编码器或 `phase` 积分。

## 8. 移植时再看硬件

读 `05-portability-layer.md` 和 `mcpwm_foc_init()` / `timer_reinit()`。此时你已经知道算法需要什么（零矢量电流、母线电压、三个比较值、死区），再决定 G 系列定时器如何提供它们。不要从 `hwconf/` 的某一块板开始抄 GPIO。

`06-minimal-first-port-slice.md` 的清单是这一步的出口。

## 建议跳过的目录

| 路径 | 原因 |
| --- | --- |
| `lispBM/`、`qmlui/` | 脚本与 UI，不在电流环 |
| `applications/` | 油门曲线，最终只是调用 `mc_interface_set_*` |
| `comm/` | 打包。知道 `mc_interface_set_current()` 被谁调用即可 |
| `imu/`、`bms.c`、`driver/` | 附件 |
| `motor/mcpwm.c` | 另一套六步。除非要对比 `COMM_MODE_INTEGRATE` 与 FOC 开环的差别 |
| `encoder/enc_as504x.c` 等具体芯片 | 选定传感器后再读一个文件 |
| `ChibiOS_3.0.5/` | 移植目标不是把 ChibiOS 搬到 STM32G |

## 读代码时的核对问题

用来判断自己有没有读错分支：

1. 电流环每个开关周期都跑吗？没有 `HW_HAS_PHASE_SHUNTS` 时，V7 半周会 return，实际是 `foc_f_zv/2`。
2. `foc_sensor_mode` 和 `sensor_mode` 谁在 FOC 里生效？只有前者。
3. 电阻检测锁定的相位会留下来吗？不会，`m_phase_override` 在结束时清除。
4. `HFI_START` 会采用 `timer_update()` 的开环角吗？不会，它看到覆盖标志就清掉并重置 HFI 计数。
5. 默认上电模式是什么？`FOC_SENSOR_MODE_SENSORLESS`，低速开环斜坡，高速 MX Lemming 观测器（默认 `MCCONF_FOC_OBSERVER_TYPE`）。
