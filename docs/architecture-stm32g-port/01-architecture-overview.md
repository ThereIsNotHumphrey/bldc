# 架构总览

## 这份固件实际跑在什么上

运行时 MCU 支持是 **STM32F4**，不是 STM32G：

- `main.c` 包含 `stm32f4xx_conf.h`，注释写明 `TIM1` / `TIM8` 给 PWM，`TIM2` 给 FOC 采样触发，`TIM5` 给通用 `timer`。
- RTOS 是树内的 `ChibiOS_3.0.5/`（`halInit()`、`chSysInit()`、`chThdCreateStatic`）。
- 外设库是 SPL 风格（`TIM_TimeBaseInit`、`ADC_Init`、`DMA_Init`），集中在 `motor/mcpwm_foc.c` 的 `mcpwm_foc_init()` / `timer_reinit()`。
- 板级差异不走一个通用 HAL 移植层，而走 `hwconf/<vendor>/<board>/hw_*.h` 里的宏，由 `hwconf/hw.h` 提供缺省值。`Makefile` 用 `hwconf/**/hw_*.h` 枚举板名（`make` / `fw_<board>`）。

产品功能（USB/CAN 协议、LispBM、QML UI、IMU、BMS、NRF）和 FOC 内核是分开的。第一版移植可以不带它们。

## 顶层模块

```text
main.c
  halInit / chSysInit
  hw_init_gpio()                  板级 GPIO，定义在各 hw_*_core.c
  irq_handlers_init()             EXTI 编码器、PVD；FOC 的 ADC 不走这里
  conf_general_init()             配置 / EEPROM 仿真
  mc_interface_init()             读 mc_configuration，按 motor_type 选实现
       |
       +-- MOTOR_TYPE_BLDC / DC --> mcpwm_init()          motor/mcpwm.c   六步梯形
       +-- MOTOR_TYPE_FOC       --> mcpwm_foc_init()      motor/mcpwm_foc.c
                                      |
                                      +-- foc_precalc_values()            motor/foc_math.c
                                      +-- timer_reinit()                  TIM1/TIM8 PWM + TIM2 采样
                                      +-- DMA2_Stream4 → mcpwm_foc_adc_int_handler()
                                      +-- 线程: "foc timer" / "foc hfi" / "foc pid"

命令与应用（不在电流环 ISR 里）
  comm/commands.c          VESC 包：设电流、检测电机
  comm/comm_can.c          CAN 上的 detect
  applications/app.c       PPM / ADC / UART / nunchuk 等，最终调用 mc_interface_set_*
  terminal.c               串口调试命令 foc_detect_apply_all 等

角度传感器
  encoder/encoder.c        按 m_sensor_port_mode 分派到 encoder/enc_*.c
  util/utils_sys.c         utils_read_hall() → READ_HALL1/2/3() 宏

参数辨识（线程上下文，不是 ISR）
  conf_general.c           conf_general_detect_apply_all_foc()
                           conf_general_measure_flux_linkage_openloop()
                           conf_general_autodetect_apply_sensors_foc()
```

`mc_motor_type`（`datatypes.h`）只有三个值：`MOTOR_TYPE_BLDC`、`MOTOR_TYPE_DC`、`MOTOR_TYPE_FOC`。双电机板（`HW_HAS_DUAL_MOTORS`）在 `mc_interface_init()` 里把类型强制成 `MOTOR_TYPE_FOC`。

## 谁拥有什么

| 职责 | 拥有者 | 说明 |
| --- | --- | --- |
| 三相 PWM 时基、死区、互补输出 | `timer_reinit()`，`motor/mcpwm_foc.c` | 中心对齐 `TIM1`（第二路 `TIM8`）。周期 `(SYSTEM_CORE_CLOCK / f_zv)`，`f_zv` 来自 `mc_configuration.foc_f_zv` |
| 占空比寄存器 | 同文件宏 `TIMER_UPDATE_DUTY_M1/M2` | 关更新后写 `CCR1/2/3`。三电阻板与两电阻板的相序不同（`HW_HAS_3_SHUNTS`） |
| PWM 开关 / 全下桥短接 | `start_pwm_hw()` / `stop_pwm_hw()` / `full_brake_hw()` | 直接操作 `TIM_SelectOCxM`，属于 F4 定时器，不可原样搬到 G 系列 |
| ADC 触发与采样缓冲 | `mcpwm_foc_init()` | `ADC1/2/3` 三重同时模式，外部触发 `ADC_ExternalTrigConv_T2_CC2`，DMA2 Stream4 环缓到 `ADC_Value[]` |
| 电流环 ISR | `mcpwm_foc_adc_int_handler()` | DMA 半传输或全传输回调（电流通道在前三个时用 HT） |
| 采样时刻 | `TIM2` CC2，`TIMER_UPDATE_SAMP` | 默认 `MCPWM_FOC_CURRENT_SAMP_OFFSET` 为 2，从计数顶点偏一点采零矢量电流。`TIM2_IRQHandler`（`irq_handlers.c`）只做 `mcpwm_foc_tim_sample_int_handler()`：产生 COM 事件并驱动 `virtual_motor_int_handler()`，**不跑电流环** |
| Clarke / Park / 反 Park / 电流 PI | `mcpwm_foc_adc_int_handler()` 与 `control_current()` | 变换是内联代码，不是单独的 `clarke()` / `park()` 函数 |
| SVPWM | `foc_svm()`，`motor/foc_math.c` | 输入归一化 αβ 电压，输出三相比较值 |
| 磁链观测器 | `foc_observer_update()` | 类型由 `foc_observer_type`（`mc_foc_observer_type`）选择 |
| 速度 PLL | `foc_pll_run()` | 每个电流环周期调用 |
| 速度环 / 位置环 | `foc_run_pid_control_speed()` / `foc_run_pid_control_pos()` | 线程 `"foc pid"`，周期由 `sp_pid_loop_rate` 决定，输出 `m_iq_set` / 速度设定，不直接写 PWM |
| 低速开环角度 | `timer_update()` | 1 ms 线程 `"foc timer"`。只在部分 `foc_sensor_mode` 下被 ISR 采纳 |
| HFI 角度 | ISR 里注入 + 线程 `hfi_update()` | 见 [03-startup-and-sensorless.md](03-startup-and-sensorless.md) |
| 霍尔换相角 | `foc_correct_hall()` | 表是 `foc_hall_table[8]`，读脚是 `utils_read_hall()` |
| 编码器角 | `encoder_read_deg()`，校正 `foc_correct_encoder()` | 初始化在 `mc_interface_init()` → `encoder_init()` |
| 旧六步换相 | `commutate()`，`motor/mcpwm.c` | 仅 `MOTOR_TYPE_BLDC`。FOC 模式不调用它 |
| 母线/相电压、分流增益、栅极使能 | `hwconf/hw.h` 与板文件 | `GET_CURRENT1()`、`GET_INPUT_VOLTAGE()`、`ENABLE_GATE()`、`FAC_CURRENT*`、`READ_HALL*` |

`irq_handlers.c` 里的 `ADC1_2_3_IRQHandler` 调用 `mc_interface_adc_inj_int_handler()`。该函数对 `MOTOR_TYPE_FOC` **直接 break**，只把注入转换中断交给 `mcpwm_adc_inj_int_handler()`（梯形波）。FOC 的快环是 DMA，不是这个 IRQ。

## 控制流（FOC，单次 PWM 周期）

```text
TIM1 中心对齐计数
    TIM2 从 TIM1 复位（timer_reinit 里 ITR0 + SlaveMode_Reset）
    TIM2 CC2 在零矢量附近触发 ADC
    DMA2_Stream4 半/全传输
        mcpwm_foc_adc_int_handler()
            读 ADC_Value → 减 foc_offsets_current → 乘 FAC_CURRENT*
            两电阻时 curr2 = -(curr0+curr1)；三电阻时按 foc_current_sample_mode 取舍
            Clarke → i_alpha, i_beta
            运行时:
                foc_observer_update()          始终更新，除非 m_phase_override
                switch (foc_sensor_mode)       选出 state.phase
                开环/手刹/相位覆盖可再改 phase
                MTPA、foc_run_fw()、电流限幅 → id_target, iq_target
                control_current(dt)
                    Park → id, iq
                    PI → vd, vq（foc_current_kp/ki）
                    解耦 foc_cc_decoupling
                    反 Park → mod_alpha_raw, mod_beta_raw
                    可选 HFI 电压注入
                    foc_svm() → TIMER_UPDATE_DUTY_M1()
            未运行时: 观测器跟踪反电动势，把 vd_int/vq_int 预置成反电动势
            foc_pll_run()、速度差分、电周期转成 6 步转速表
            mc_interface_mc_timer_isr()   故障、限幅、采样记录，不是换相
```

速度/位置指令从外面进来：

```text
app_* / commands / Lisp
    mc_interface_set_current|duty|pid_speed|pid_pos|openloop_*()
        mcpwm_foc_set_*()
            写 m_iq_set / m_duty_cycle_set / m_speed_pid_set_rpm / m_control_mode
                电流环 ISR 在下一拍使用
    "foc pid" 线程把速度/位置误差变成 m_iq_set（电流模式）或速度设定
```

`mc_control_mode`（`datatypes.h`）包括 `CONTROL_MODE_DUTY`、`SPEED`、`CURRENT`、`CURRENT_BRAKE`、`POS`、`HANDBRAKE`、`OPENLOOP`、`OPENLOOP_PHASE`、`OPENLOOP_DUTY`、`OPENLOOP_DUTY_PHASE`、`NONE`。转矩指令在这套代码里就是 q 轴电流 `m_iq_set`，没有单独的转矩单位环。

## 状态数据放在哪

算法状态几乎都在 `motor_all_state_t` / `motor_state_t`（`motor/foc_math.h`）：

- 本拍电气量：`i_alpha/i_beta`、`id/iq`、`vd/vq`、`vd_int/vq_int`、`mod_d/mod_q`、`mod_alpha_raw/mod_beta_raw`、`phase`、`svm_sector`
- 角度源：`m_phase_now_observer`、`m_phase_now_encoder`、`m_ang_hall`、`m_hfi.angle`、`m_openloop_angle`、`m_phase_now_observer_override`
- 观测器状态：`observer_state` 的 `x1`、`x2`（静止轴磁链）、`lambda_est`
- 配置指针：`m_conf` → `mc_configuration`（`datatypes.h`，体积很大，含 FOC、BLDC、应用限幅、BMS）

板级宏和 `mc_configuration` 是算法与硬件的接口。移植时优先保留这个结构的 **FOC 子集**，而不是把 `mcpwm_foc.c` 整文件丢进 STM32G 工程。

## 明确不在 FOC 快环里的东西

- `lispBM/`、`qmlui/`、`imu/`、`bms.c`、`driver/nrf/`、`driver/lora/`、`blackmagic/`
- `comm/packet.c` 只组包，不产生 PWM
- `motor/virtual_motor.c` 是无硬件时的电机模型，由 `TIM2` 中断调用；接虚拟电机时 `control_current()` 跳过真实 `start_pwm_hw()`
