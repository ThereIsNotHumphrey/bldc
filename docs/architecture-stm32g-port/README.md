# STM32G 第一版 FOC 移植：架构阅读包

本目录是对 **本仓库当前树**（VESC / vedderb 风格 BLDC/FOC 固件，`ThereIsNotHumphrey/bldc`）的只读架构说明，给电机控制教学和 **STM32G 系列上的最小 FOC 移植** 用。它不改控制算法，也不声称已经能在 STM32G 上编译。

分析基准：`master` @ `1aa0566e`（`Merge pull request #956 from lukash/madgwick-fix-zero`）。函数名、枚举、路径都以该树为准。

## codegraph-ua

在本环境的 agent skills、`.cursor/skills`、用户 skills 和仓库内 **没有** 名为 `codegraph-ua` / `codegraph` 的 `SKILL.md`。下面的模块图和调用链是对照源码手工整理的，不是 codegraph 工具的输出。文中每条控制路径都标了文件和符号，便于回源码核对。

## 文档

| 文件 | 内容 |
| --- | --- |
| [01-architecture-overview.md](01-architecture-overview.md) | 顶层模块、数据/控制流、PWM / ADC / 换相 / 观测器的归属 |
| [02-foc-core-map.md](02-foc-core-map.md) | Clarke/Park、电流环、速度/转矩、SVPWM、观测器、HFI、开环、编码器/霍尔 |
| [03-startup-and-sensorless.md](03-startup-and-sensorless.md) | 本 fork 里电机如何起步：开环斜坡、HFI、霍尔、编码器、旧 BLDC 梯形波 |
| [04-parameter-id-vs-rotor-angle.md](04-parameter-id-vs-rotor-angle.md) | “检测电机”量的是 Rs / L / ψf，不是转子角 |
| [05-portability-layer.md](05-portability-layer.md) | `hwconf` / 定时器 / ADC / IRQ 与可移植算法的边界 |
| [06-minimal-first-port-slice.md](06-minimal-first-port-slice.md) | STM32G v1：电流环 + SVPWM + 一种角度源 |
| [07-learning-roadmap.md](07-learning-roadmap.md) | 只学 FOC 骨架时的阅读顺序 |

## 一句话结论

FOC 运行时在 `motor/mcpwm_foc.c` 的 DMA 回调 `mcpwm_foc_adc_int_handler()` 里完成采样、Clarke、选角、`control_current()`（Park、PI、反 Park、`foc_svm()`、写 `TIMx->CCR`）。角度来自 `mc_configuration.foc_sensor_mode`（`datatypes.h` 的 `mc_foc_sensor_mode`），默认是 `FOC_SENSOR_MODE_SENSORLESS`：低速用 `timer_update()` 里的开环角度覆盖观测器，高速用 `foc_observer_update()`。本树 **没有名为 IPD 的函数**。`conf_general_detect_apply_all_foc()` 测的是电阻、电感、磁链和电流上限，不产生无感启动用的转子电角度。
