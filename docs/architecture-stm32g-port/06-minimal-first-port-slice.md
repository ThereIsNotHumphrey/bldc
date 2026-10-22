# STM32G v1 最小切片

目标：一块功率板能做 **电流闭环 + SVPWM + 一种角度**。角度二选一，不要第一版就上无感观测器 + HFI。

推荐顺序：

1. **开环电流矢量**（`CONTROL_MODE_OPENLOOP` / `OPENLOOP_PHASE`）验证采样、PI 和 PWM。
2. 再把角度换成 **霍尔或增量编码器**。
3. 无感观测器和开环斜坡作为 v2。HFI / IPD 类模糊分辨作为 v3。

本树里对应的可运行子集是：`foc_svm()` + `control_current()` 的 PI/反 Park 部分 + ISR 里 Clarke 和角度 switch 的一个 case。检测电机、弱磁、MTPA、音频、双电机、梯形波都不在 v1。

## 切下来的模块

| 顺序 | 从本树带走 | 在 G 系列新写 |
| --- | --- | --- |
| 1 | `foc_svm()`，`util/utils_math.c` 中它用到的截断函数 | 高级定时器：中心对齐、三相互补、死区、影子寄存器。接口只做“写入三个比较值” |
| 2 | 无 | ADC：在低侧零矢量采样两相或三相电流；完成回调 |
| 3 | Clarke 内联式（`mcpwm_foc.c` 3298–3306 行）和 Park/反 Park（`control_current` 4624–4721 行） | 原始值 → 安培：零点、极性、分流增益。先用常量，不要移植整份 `mcpwm_foc_dc_cal()` |
| 4 | 电流 PI 与 d/q 电压圆限幅（同函数 4630–4695 行）。解耦先固定 `FOC_CC_DECOUPLING_DISABLED` | 母线电压进 `v_bus`，否则归一化 `1.5/v_bus` 无意义 |
| 5 | 角度：开环积分 **或** `foc_correct_hall()` + `utils_read_hall_hw()` **或** 一个 `encoder_read_deg()` | 霍尔 GPIO 或编码器定时器。只做一个 |
| 6 | `mc_interface_set_current()` 的语义：写 `iq_target` | 一个非 ISR 的设定入口（调试器变量或 UART 浮点即可） |

明确后移：

- `foc_observer_update()`、`timer_update()` 的开环覆盖、`hfi_*`
- `foc_run_pid_control_speed/pos`、`foc_run_fw`、MTPA
- `conf_general_detect_apply_all_foc()`（R/L/ψ 可以先用万用表和已知电机手填 `foc_motor_r/l`）
- `start_pwm_hw` 里的 F4 `TIM_SelectOCxM`；用新定时器的 MOE/BDTR 等价行为代替
- `HW_HAS_DUAL_MOTORS`、`virtual_motor`（除非用来做离线测试）
- `MOTOR_TYPE_BLDC` 与 `commutate()`

## 实施清单

按这个顺序做。每一步有可以观察的现象，再做下一步。

1. **PWM 空转。** 三相互补、死区非零、栅极驱动可关断。比较值都设在半周期，用示波器看占空 50% 且下管/上管不直通。对应本树 `timer_reinit()` + `stop_pwm_hw()` 的安全态：初始化结束时 PWM 输出被关掉，而不是 50% 直通。上电默认应是 `stop`（所有相高阻或全下桥，与驱动芯片的安全电平一致），而不是立刻 `foc_svm`。
2. **零矢量采样。** 电机不转、栅极关闭或 50% 但无电流时，两路或三路电流原始值稳定在中点附近。记录为零点。本树字段名是 `foc_offsets_current[]`。
3. **开环固定相位。** 复现 `mcpwm_foc_set_openloop_phase()`：`phase` 恒定，`iq_target` 从小电流加上去，`id_target = 0`。转子应锁在一个位置。电流反馈 `iq` 应跟上目标。这证明 Clarke、Park、PI、SVPWM、采样极性一致。极性反了时电机会抖或电流符号和电压相反。
4. **开环旋转。** 复现 `m_openloop_angle += dt * ω`（`CONTROL_MODE_OPENLOOP`）。给定一个小的电转速和电流，转子跟着转。转速不对通常是极对数没有算进 ERPM，或 `dt` 用的不是真实控制周期。本树 ERPM 是电转速：`RPM2RADPS_f` / `RADPS2RPM_f` 在 `util/utils_math.h`，换算是 `rad/s = rpm * 2π / 60`。
5. **选定角度源替换开环相位。**
   - 霍尔：移植 `foc_correct_hall()`。表可以先手填六个扇区（0, 200/6, …），再考虑 `mcpwm_foc_hall_detect()`。`foc_hall_table` 元素 `< 201` 才有效。
   - 编码器：每个控制周期读机械角，乘 `foc_encoder_ratio`（极对数），减 `foc_encoder_offset`。v1 可用 `FOC_SENSOR_MODE_ENCODER` 的“有索引”分支，并把 `foc_sl_erpm` 设得很高，从而 `foc_correct_encoder()` 一直返回编码器角、不切观测器。
6. **电流模式带载。** `iq_target` 改号，转子换向。保持 `id_target = 0`（与本树在霍尔/编码器/无感分支里对非覆盖相位的处理一致）。
7. **限幅。** 加上 `l_current_max` 和电压圆（`max_v_mag`）。缺电压圆时 PI 积分会在占空饱和后缠绕，本树靠 `vd_int/vq_int` 与 `vd/vq` 一起截断来抗饱和。
8. **停机。** 复现 `mcpwm_foc_release_motor()` / `stop_pwm_hw()`：电流设 0，关闭互补输出。不要只把比较值写成 0，那在互补 PWM 里可能是满下桥或满上桥，取决于极性宏 `INVERTED_TOP_DRIVER_INPUT`。

## v1 故意不做时，对应代码不要“半开”

- 观测器函数可以编译进来，但 `foc_sensor_mode` 不要设 `SENSORLESS`，否则 `timer_update()` 的开环覆盖和一份未调的 `foc_observer_gain` 会在低速抢角度。
- 不要定义 `HW_HAS_3_SHUNTS` 除非三路分流都接到 ADC。错误的 Clarke 会让 `id` 出现虚假正弦。
- `foc_cc_decoupling` 保持关闭，直到 `foc_motor_l` 和 `foc_motor_flux_linkage` 可信。错误的前馈会表现为转速一高电流环振荡。
- HFI 的 `CURRENT_FILTER_OFF()` 会改变硬件带宽。没有对应电路就不要打开 `do_hfi` 路径。

## 验收（仍在实验台，不是本仓库的测试）

- 固定相位：`iq` 阶跃到小电流，无持续振荡，转子锁定。
- 开环旋转：电流幅值大致恒定，相电流正弦，频率等于指令电频率。
- 霍尔或编码器：手转转子时估计角单调、每机械转变化 `极对数` 个电周期；上电后正反 `iq` 都能平滑起步，而不是要先甩一下。
- 任一故障（过流、驱动 nFAULT、母线欠压）能在驱动芯片或比较器侧硬关 PWM，而不是只在循环里把占空写成 0。本树的硬关断是定时器 BRK（`timer_reinit()` 里 `HW_USE_BRK`），G 系列用对应的 break 输入。
