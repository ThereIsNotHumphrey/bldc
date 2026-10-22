# 可移植层：STM32G 第一版该留什么、重写什么

本仓库没有 MCU 抽象层。可移植的是 **纯计算**，不可移植的是 **直接写 STM32F4 定时器、ADC、DMA 和 ChibiOS 中断** 的代码。STM32G 系列的定时器和 ADC 与 F4 不同（G4 仍有高级定时器和注入组，G0 资源更紧）。下面按 **行为** 划分边界，不编造一套 STM32G 寄存器流程。

## 边界图

```text
可保留（算法，几乎不碰寄存器）
  motor/foc_math.c          观测器、foc_svm、PLL、速度/位置 PID、霍尔融合、HFI 角度校正
  motor/foc_math.h          motor_all_state_t / motor_state_t / observer_state / hfi_state_t
  util/utils_math.c         sincos、atan2、角度差、饱和
  datatypes.h 里的 FOC 枚举与 mc_configuration 的 FOC 字段
  conf_general.c 中的测量公式与 conf_general_calc_apply_foc_cc_kp_ki_gain()
      （调用 mc_interface 和延时，要等 PWM 层能跑再搬）

必须重写（F4 + ChibiOS + 板级宏）
  mcpwm_foc_init() 后半段   ADC 三重模式、DMA2_Stream4、触发 T2_CC2
  timer_reinit()            TIM1/TIM8 中心对齐互补 PWM，TIM2 从 TIM1 复位
  TIMER_UPDATE_DUTY_*       TIM1->CCR*，更新抑制位 TIM_CR1_UDIS
  start_pwm_hw/stop_pwm_hw/full_brake_hw
  mcpwm_foc_adc_int_handler 的前半：读 ADC_Value、GET_CURRENT*、V7/双电机
  irq_handlers.c            ADC1_2_3（梯形波）、TIM2、编码器 EXTI、PVD
  hwconf/**                 引脚、分流极性、死区、栅极驱动、霍尔脚
  ChibiOS_3.0.5 线程        "foc timer" / "foc hfi" / "foc pid" 可改成任意 1 ms / 0.5 ms / PID 周期任务

按 v1 范围整块不搬
  motor/mcpwm.c             六步
  lispBM/ qmlui/ imu/ bms.c comm/ 的完整协议
  encoder/enc_* 里除你选定的那一种传感器
  applications/             PPM、ADC 油门等，可在电流环通了之后再接一个设定值
```

`foc_math.c` 包含 `hw.h`，但算法函数本身用的是 `motor_all_state_t` 和 `mc_configuration`。链接时需要一个最小的 `hw.h`（空的 `HW_HAS_*`），而不是把某块 VESC 的原理图宏带进来。`ONE_BY_SQRT3`、`TWO_BY_SQRT3`、`RPM2RADPS_f`、`RADPS2RPM_f` 在 `util/utils_math.h`。

## 板级宏：新 MCU 上要重新提供的行为

定义集中在各板 `hw_*_core.h`，缺省在 `hwconf/hw.h`。FOC ISR 真正依赖的有：

| 宏 / 符号 | 使用处 | v1 需要的语义 |
| --- | --- | --- |
| `GET_CURRENT1/2/3()` | ISR 开头 | 分流 ADC 原始值，0…满量程。`INVERTED_SHUNT_POLARITY` 会用 `4095 - raw`。G 系列换了 ADC 位数就不能再写死 4095，应改成满量程减 raw |
| `FAC_CURRENT1/2/3` | 同一处 | 原始值到安培的比例，含分流电阻和运放增益。这是板级标定，不是电机参数 |
| `foc_offsets_current[3]` | 配置字段，不是宏 | 零电流时的 ADC 中点。`mcpwm_foc_dc_cal()` 测量它 |
| `GET_INPUT_VOLTAGE()` | ISR，滤成 `v_bus` | 母线电压伏特 |
| `ENABLE_GATE()` / `DISABLE_GATE` 一类 | `mcpwm_foc_init()` | 栅极驱动使能。名字以具体板为准 |
| `HW_DEAD_TIME_NSEC` | `timer_reinit()` → `conf_general_calculate_deadtime()` | 死区纳秒。G 系列高级定时器的死区寄存器编码不同，函数实现要换，**数值**可留 |
| `READ_HALL1/2/3()` | `utils_read_hall_hw()` | 仅霍尔角度源需要 |
| `ADC_V_L1_VOLTS` 等 | `update_valpha_vbeta()` | 相电压采样。v1 若没有相电压电阻，可只用调制反推 αβ 电压（函数后半已有这条路径） |
| `CURRENT_FILTER_ON/OFF()` | HFI | 硬件电流低通的开关。没有该电路就定义成空宏 |
| `HW_HAS_3_SHUNTS` | Clarke 与 CCR 相序 | 两电阻和三电阻是两套采样假设，v1 只实现你板上的那一种 |
| `HW_HAS_PHASE_SHUNTS` | 能否在 V7 采样 | 低边分流通常只能在低侧导通的零矢量采样，对应现在的“非 V7 就 return” |
| `SYSTEM_CORE_CLOCK` | ARR 计算 | 新系统的定时器时钟，注意 G4 APB 倍频后定时器时钟常常是内核时钟的 2 倍，不能沿用 F4 的 `SYSTEM_CORE_CLOCK / f_zv` 而不核对 |

`hw_init_gpio()` 与 `hw_setup_adc_channels()` 在每块 `hw_*_core.c`。它们是 SPL 的 `ADC_InjectedChannelConfig` / GPIO，只能当 **通道清单** 读，不能编译进 G 系列。

## `mcpwm_foc.c` 里算法和寄存器缠在一起的点

这些函数是移植时要切开的缝，而不是整段删除：

1. **`mcpwm_foc_adc_int_handler()`**  
   前半（约 2875–3254 行）是双电机、V7、分流选择、DMA 缓冲。后半（Clarke、观测器、`switch (foc_sensor_mode)`、`control_current()`、PLL）是算法。建议抽一个  
   `foc_step(motor, ia, ib, ic, vbus, dt)`  
   让 G 系列的 ADC 回调只负责把安培和母线电压送进去。本仓库还没有这个函数，这是移植时的新边界，不是现成 API。

2. **`control_current()` 的最后 40 行**  
   `top = TIM1->ARR`，`foc_svm(..., top, &duty)`，然后 `TIMER_UPDATE_DUTY_M1`。`foc_svm()` 已把比较值限制在 `0 … top * max_mod`。新驱动应提供 `pwm_set_compare(duty1, duty2, duty3)`，在更新事件之前写入影子寄存器。中心对齐、互补输出、死区要在驱动里打开，否则 `foc_svm()` 的 0 和 `top` 没有“全关/全开”含义。

3. **采样时刻**  
   现在是：`TIM1` 中心对齐 → `TIM2` 复位同步 → `TIM2->CCR2 = samp/2` → ADC 在该比较点触发 → DMA 完成再跑控制。目的是在低侧零矢量（电流流过三个下管和分流）采样。G 系列可以用高级定时器的 CC4 或 TRGO 触发 ADC，只要满足同一时序：**采完、算完、在下一个更新事件前写 CCR**。`MCPWM_FOC_CURRENT_SAMP_OFFSET`（默认 2 个计数）是相对顶点的余量。

4. **`timer_update()` / `hfi_update()` / `pid_thread`**  
   三个 ChibiOS 线程。算法可留，调度改成裸机定时器或任何 RTOS。`timer_update` 的 `dt` 固定写 0.001，必须真的 1 ms 调用一次，开环斜坡时间才和 `foc_sl_openloop_time_*` 一致。HFI 线程是 500 µs 量级，只服务经典 FFT HFI；若 v1 不做 HFI，这个线程可以不创建（`mcpwm_foc_init()` 里 `chThdCreateStatic(..., hfi_thread, ...)`）。

5. **`mcpwm_foc_dc_cal()`**  
   逐相打开 PWM 测电流和相电压零点，结果写入 `foc_offsets_current` / `foc_offsets_voltage`。逻辑可留，里面的 `TIM_SelectOCxM` 和 `TIMER_UPDATE_DUTY_M1(ARR/2, …)` 要换成新驱动的“50% 占空、只开一相”。

## 建议保留的配置字段（v1 子集）

来自 `mc_configuration`，名字保持与本树一致，便于以后对照 VESC Tool：

- 电机：`foc_motor_r`、`foc_motor_l`、`foc_motor_ld_lq_diff`、`foc_motor_flux_linkage`
- 电流环：`foc_current_kp`、`foc_current_ki`、`foc_cc_decoupling`、`l_current_max`、`l_current_min`
- PWM：`foc_f_zv`、`foc_overmod_factor`、`l_max_duty`
- 角度：`foc_sensor_mode`（v1 只实现其中一个）、`foc_observer_type`、`foc_observer_gain`、`foc_observer_gain_slow`、`foc_observer_offset`、`foc_pll_kp`、`foc_pll_ki`
- 无感开环：`foc_openloop_rpm`、`foc_sl_openloop_hyst`、`foc_sl_openloop_time_lock`、`foc_sl_openloop_time_ramp`、`foc_sl_openloop_time`
- 若选霍尔：`foc_hall_table[8]`、`foc_sl_erpm`、`foc_hall_interp_erpm`、`m_hall_extra_samples`
- 若选编码器：`foc_encoder_offset`、`foc_encoder_ratio`、`foc_encoder_inverted`、`m_sensor_port_mode`

可以不搬的字段：BMS、`m_drv8301_*`、音频、HFI 电压（除非做 HFI）、弱磁、MTPA、双电机、`foc_offsets_cal_mode` 的在线跟踪位（`timer_update` 里 `foc_offsets_cal_mode & (1<<2)`）。

## 和 STM32G 能力的对应关系（指导，不是本树里的代码）

| 本树在 F4 上的做法 | 移植时要在 G 系列重做的行为 |
| --- | --- |
| TIM1 三相互补 + BDTR 死区 + 刹车 | 一块带互补输出和死区的高级定时器。G0 上往往没有与 TIM1 等价的三相互补组，第一块板子更合理的是带高级定时器的 G4 |
| TIM8 第二台电机或并联 | v1 不要。删掉 `HW_HAS_DUAL_MOTORS` 分支 |
| ADC1/2/3 同时采样，`ADC->CDR`，DMA2 Stream4 | G4 的 ADC 是另一套（常见为 12-bit、注入组或规则组 DMA）。需要同时得到两路或三路电流；做不到同时采样时，要接受错相并在文档里记一拍误差 |
| 注入中断 `ADC1_2_3_IRQHandler` 只服务梯形波 | FOC 不必复刻这个 IRQ。把完成回调接到抽出的 `foc_step` |
| ChibiOS `nvicEnableVector` | 用你的启动文件和 NVIC |
| `chThdSleepMilliseconds` 包住的检测流程 | 检测可以第二阶段再移植；电流环 v1 不依赖 `conf_general.c` 的线程睡眠 |

不要把 `ChibiOS_3.0.5/` 或 `stm32f4xx` SPL 放进 G 系列工程再宏替换。寄存器布局和 DMA 通道表对不上，`timer_reinit()` 会成为错误来源。

## 虚拟电机

`motor/virtual_motor.c` 的 `virtual_motor_int_handler(v_alpha, v_beta)` 在 `mcpwm_foc_tim_sample_int_handler()` 里调用。`virtual_motor_is_connected()` 为真时，编码器角可由模型提供，且 `control_current()` 不调用 `start_pwm_hw()`。它适合在 PC 或新 MCU 上先验证观测器和 SVPWM 数值，再接功率级。模型本身含简化的逆 Park/Clarke（`run_virtual_motor_park_clark_inverse`），不是硬件驱动。
