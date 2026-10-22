# 启动与无感路径

本 fork **没有** `ipd`、`IPD` 或 “initial position detection” 符号（全树检索无匹配）。初始电角度来自下面几条互斥路径，由 `mc_configuration.foc_sensor_mode` 选择。默认 `MCCONF_FOC_SENSOR_MODE` = `FOC_SENSOR_MODE_SENSORLESS`（`motor/mcconf_default.h`）。

角度在 `mcpwm_foc_adc_int_handler()` 的 `switch (conf_now->foc_sensor_mode)` 里选定，随后还可能被手刹、开环指令或 `m_phase_override` 覆盖。

## 总表

| `foc_sensor_mode` | 低速角度从哪来 | 高速角度 | 启动时是否先转一圈 |
| --- | --- | --- | --- |
| `FOC_SENSOR_MODE_SENSORLESS` | `timer_update()` 的开环覆盖 `m_phase_now_observer_override` | `foc_observer_update()` → `m_phase_now_observer` | 是，开环锁定/斜坡/保持，不是脉冲测角 |
| `FOC_SENSOR_MODE_HALL` | `foc_correct_hall()` + `foc_hall_table` | 同上，`|rpm| >= foc_sl_erpm` 后交观测器 | 否。表必须事先标定 |
| `FOC_SENSOR_MODE_ENCODER` | 有索引：编码器角。无索引：`m_phase_now_encoder_no_index` 开环转，直到 `encoder_index_found()` | `foc_correct_encoder()`，阈值 `foc_sl_erpm` | 仅在索引未找到时开环转 |
| `FOC_SENSOR_MODE_ENCODER_AB` | 第一次同步前：开环覆盖或观测器。`|speed| >= foc_sl_erpm` 时 `encoder_set_deg()` 把编码器拨到观测器角 | 编码器与观测器按 `foc_correct_encoder()` 切换 | 低速仍会走 `timer_update()` 的开环计时 |
| `FOC_SENSOR_MODE_HFI` 及 `_V2` `_V3` `_V4` `_V5` | `m_hfi.angle`。完成前 `iq_target = 0` | `foc_correct_encoder(observer, hfi.angle, …, foc_sl_erpm_hfi)` | 静止注入，不靠开环拖转。180° 模糊由 `foc_hfi_amb_mode` 解 |
| `FOC_SENSOR_MODE_HFI_START` | 先 HFI，样本够后把观测器状态设到 HFI 角，之后 `phase = m_phase_now_observer`，并关掉 HFI | 观测器 | 一次静止辨向，然后无感观测器 |

另外，与传感器模式无关的 **指令开环**：`CONTROL_MODE_OPENLOOP*` 和 `CONTROL_MODE_HANDBRAKE`。参数辨识使用这些指令，见下一篇。

`FOC_SENSOR_MODE_SENSORLESS` 的开环覆盖 **不会** 被 HFI / 霍尔 / 带索引编码器分支采用。`timer_update()` 仍会计算 `m_phase_observer_override`，但 ISR 只在 `SENSORLESS` 和 `ENCODER_AB`（以及编码器无索引时的另一变量）里把它写成 `phase`。`HFI_START` 看到该标志会清零 HFI 计数并立刻把标志清掉，**不用**那个开环角。

## 1. 无感：开环斜坡再切观测器

配置字段（`mcconf_default.h` 默认值）：

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `foc_openloop_rpm` | 1500 ERPM | 开环电转速上限 |
| `foc_openloop_rpm_low` | 0 | 电流很小时，上限乘的比例 |
| `foc_sl_openloop_hyst` | 0.1 s | 转速低于开环上限持续这么久才 **进入** 开环 |
| `foc_sl_openloop_time_lock` | 0 s | 进入后先锁住（开环转速 0） |
| `foc_sl_openloop_time_ramp` | 0.1 s | 开环转速从 0 拉到目标 |
| `foc_sl_openloop_time` | 0.05 s | 拉到顶后再保持 |
| `foc_sl_openloop_boost_q` | 0 | 开环期间附加的 q 轴电流 |
| `foc_sl_openloop_max_q` | -1（不限制，除非大于 `cc_min_current`） | 开环 q 轴电流帽 |
| `foc_sl_erpm` | 3500 | 霍尔/编码器切到纯观测器的阈值，不是这条开环斜坡的转速 |

实现：`timer_update()`（`motor/mcpwm_foc.c`，1 ms）。

1. 用 `|iq_filter| + foc_sl_openloop_boost_q` 在 `foc_openloop_rpm_low * foc_openloop_rpm` 和 `foc_openloop_rpm` 之间映射出 `openloop_rpm_max`。
2. 非编码器模式按剩余时间把转速分成锁、斜坡、保持。编码器两种模式跳过锁和斜坡，直接用上限转速去积分“无索引角度”。
3. `|m_pll_speed| < openloop_rpm_max` 时累加 `m_min_rpm_hyst_timer`，否则递减。制动或占空近 0 时两个计时器清零并取消覆盖。
4. 滞环到点且 `m_min_rpm_timer ≈ 0`：装载 `t_lock + t_ramp + t_const`，并把覆盖角 **跳 60°**（`M_PI / 3`，方向跟 `duty_now`）。注释写明：卡住时观测器常偏 90°，用 60° 把序列拉开。
5. 计时期间 `m_phase_observer_override = true`，覆盖角按 `±openloop_rpm` 积分。同时用 `sincos(override + sign(duty)*π/4)` 乘 `foc_motor_flux_linkage` 得到 `m_observer_x1_override` / `x2_override`，留给退出开环时塞进观测器。
6. 计时结束：标志清掉，覆盖角被设回当前观测器角。

ISR 在 `FOC_SENSOR_MODE_SENSORLESS`：

```text
if (m_phase_observer_override)
    phase = m_phase_now_observer_override
    observer.x1/x2 = overrides
    iq += sign(iq) * foc_sl_openloop_boost_q
else
    phase = m_phase_now_observer
id_target = 0          // 除非相位被检测函数覆盖，或模式是 OPENLOOP_PHASE
```

观测器在覆盖期间 **仍然每个控制周期被调用**（除非 `m_phase_override`），只是状态被开环角强行改写，所以退出时不是从零开始收敛。

这不是 IPD：固件并不测量转子停在哪，而是规定一个电角度并把电流矢量拖着走，直到反电动势够观测器跟踪。`foc_motor_flux_linkage` 只用来给观测器一个初始矢量长度。

## 2. HFI 静止测角（本树里最接近 IPD 的机制）

`do_hfi` 在 `control_current()`：模式是 `HFI` / `HFI_V2`…`HFI_V5`，或 `HFI_START` 且不是制动、`|iq_target| > cc_min_current`；并且没有 `m_phase_override`；并且 `|m_speed_est_fast|` 低于 `foc_sl_erpm_hfi` 乘 1.5（上一拍已是 HFI 则乘 1.8）。

完成判据：`m_hfi.est_done_cnt >= foc_hfi_start_samples`。未完成时 `iq_target = 0`，电机不产生转矩电流，只注入辨识电压。`HFI_START` 一旦完成就把 `do_hfi` 置假，之后不再注入。

注入电压：完成前 `foc_hfi_voltage_start`，完成后在 `foc_hfi_voltage_run` 与 `foc_hfi_voltage_max` 间随 `|iq|` 映射，并受母线和当前占空限制。

未完成时，注入波形被改写：

- `foc_hfi_amb_mode == FOC_AMB_MODE_SIX_VECTOR` → 临时用经典 `FOC_SENSOR_MODE_HFI`（`utils_tab_cos_32_1` 旋转电压）
- 否则 → 临时用 `FOC_SENSOR_MODE_HFI_V5`

`mc_foc_hfi_amb_mode`（ambiguity，不是制动）：

| 值 | `hfi_update()` 在初始样本阶段做什么 |
| --- | --- |
| `FOC_AMB_MODE_SIX_VECTOR` | 对 `m_hfi.buffer` 做 bin1/bin2。bin2 的角度/2 是转子角（凸极二次谐波）。与 bin1（饱和方向）差超过 90° 则 `flip_cnt++`。样本数到达后，若翻转次数过半，角度加 π |
| `FOC_AMB_MODE_D_SINGLE_PULSE` | 先零 d 轴电流，再 `id_target = foc_hfi_amb_current`，再回到 0。比较正电流与零电流下的电感样本，差值为正则角度加 π。阈值再加上 `foc_hfi_amb_tres`（百分数） |
| `FOC_AMB_MODE_D_DOUBLE_PULSE` | 正、负 d 轴电流各一段（`±foc_hfi_amb_current`），同样用差值决定是否加 π |

经典 `FOC_SENSOR_MODE_HFI` 在整个低速区都靠这段 FFT（`hfi_thread` 每 500 µs）。`HFI_V2`/`V3`/`V4`/`V5` 在 `est_done` 之后不再走 FFT，改由 ISR 里的 `foc_hfi_adjust_angle()` 积分角度；此时 `foc_correct_encoder()` 把低速的“编码器”标志设为真，函数才会真的修正角度，高速时改为把 `m_hfi.angle` 钉在观测器上。

`HFI_START` 在 FFT 样本刚攒够时：

```text
utils_fast_sincos_better(angle_bin_2)
observer.x1 = cos * foc_motor_flux_linkage
observer.x2 = sin * foc_motor_flux_linkage
```

然后 ISR 使用观测器角。这是 **一次性初始角**，之后不再依赖凸极，前提是磁链参数 `foc_motor_flux_linkage` 已经可用。

高转速：`|speed| > foc_sl_erpm_hfi` 时 `observer_zero_time` 清零，HFI 角被观测器角替换；低于该转速的时间累计超过 `foc_hfi_obs_ovr_sec` 才重新信任 HFI。未运行（`MC_STATE` 不是 `RUNNING`）且 `|m_pll_speed| < foc_hfi_reset_erpm` 时清 `est_done_cnt` 和 `flip_cnt`，下次启动重做模糊分辨。

电感测量复用了这条注入，但只读 FFT 的 bin0/bin2 幅值来算 `Ld`/`Lq`，不把 `angle_bin_2` 存进配置。见下一篇。

## 3. 霍尔

`FOC_SENSOR_MODE_HALL` 从第一拍就用 `foc_correct_hall()`。没有单独的开环起步分支（`id_target` 同样被清零，除非相位覆盖）。

若 `foc_hall_table` 仍是默认 255，`foc_correct_hall()` 里 `ang_hall_int < 201` 失败，退回传入的观测器角。所以上电直接转需要先跑 `mcpwm_foc_hall_detect()` 或 `conf_general_autodetect_apply_sensors_foc()`。

检测本身是开环：`m_phase_override = true`，`m_id_set` 斜坡到给定电流（d 轴），相位从 0 扫到 360° 再扫回来，每个霍尔组合累加 sin/cos。这张表是 **霍尔码 → 电角度**，不是电阻测量的副产品。

`foc_sl_erpm`（默认 3500）以上，即使模式是霍尔，输出也切到观测器。`foc_hall_interp_erpm`（默认 500）以下不做扇区内插。

## 4. 编码器

`FOC_SENSOR_MODE_ENCODER`：

- `encoder_index_found()` 或虚拟电机已连接：`foc_correct_encoder()`。
- 否则：`phase = m_phase_now_encoder_no_index`。该变量在 `timer_update()` 里按开环转速积分，用来拖动转子直到索引脉冲。编码器模式下开环转速 **不经过** lock/ramp 塑形，一进来就是 `openloop_rpm_max`。

`FOC_SENSOR_MODE_ENCODER_AB`：没有索引脚。低速若 `m_phase_observer_override` 则与无感相同（开环角 + q 轴 boost）；否则用观测器。一旦 `|m_speed_est_fast| >= foc_sl_erpm`，用观测器角度反推编码器零位并 `encoder_set_deg()`。之后走 `foc_correct_encoder()`。

偏移、极对比例、方向的标定函数是 `mcpwm_foc_encoder_detect()`，以及磁链开环测量顺带算出的 `enc_offset` / `enc_ratio` / `enc_inverted`（`conf_general_measure_flux_linkage_openloop()` 后半段，电机已经在转并开始自由减速时，把编码器增量与反电动势相位 `mcpwm_foc_get_phase_bemf()` 比较）。那是传感器安装角，不是“Rs 测出来的转子位置”。

## 5. 指令开环与手刹

这些不看 `foc_sensor_mode`，在 switch 之后执行：

- `CONTROL_MODE_HANDBRAKE`：`phase = 0`，转子被吸到固定轴。
- `CONTROL_MODE_OPENLOOP` 与 `OPENLOOP_DUTY`：积分 `m_openloop_angle`。
- `CONTROL_MODE_OPENLOOP_PHASE` 与 `OPENLOOP_DUTY_PHASE`：使用 `m_openloop_phase`。
- `m_phase_override`：使用 `m_phase_now_override`。检测函数用它，优先级最高。

`mc_interface_set_openloop_current()` 是磁链测量的启动手段：先 200 ms 把电流从 0 拉到目标且 rpm 仍为 0（锁转子），再按 `erpm_per_sec` 增加开环转速直到占空达到设定（`conf_general_measure_flux_linkage_openloop()`）。

## 6. 旧梯形波启动（`MOTOR_TYPE_BLDC`）

与 FOC 平行，v1 移植可以不带。入口 `mcpwm_init()`，换相 `commutate()` / `set_next_comm_step()`。

- `sensor_mode == SENSOR_MODE_SENSORLESS`，或 `SENSOR_MODE_HYBRID` 且 `|rpm| > hall_sl_erpm`：`update_sensor_mode()` 把 `sensorless_now` 设为真。
- 无感换相模式 `comm_mode`：`COMM_MODE_INTEGRATE`（过零积分，注释称更稳、参数更多）或 `COMM_MODE_DELAY`（注释称类似航模 ESC，启动更差）。API：`mcpwm_set_comm_mode()`。
- 启动增压占空在换相 ISR 路径里使用 `cc_startup_boost_duty`（`mcpwm.c` 约 1999 行）。

FOC 固件的“像 VESC 一样启动”若 `motor_type == MOTOR_TYPE_FOC`，指的是第 1–5 节，不是 `commutate()`。

## 配置与命令从哪触发

| 动作 | 符号 |
| --- | --- |
| 上电选择 FOC 或梯形 | `mc_interface_init()` 读 `motor_type` |
| 运行中改传感器模式 | `mc_interface_set_configuration()` → `mcpwm_foc_set_configuration()`，会 `foc_precalc_values()` 并可能 `update_hfi_samples()` |
| 终端无感/霍尔/编码器检测 | `terminal.c`：`foc_sensors_detect_apply`、`foc_detect_apply_all` |
| 协议 | `COMM_DETECT_MOTOR_PARAM`、`COMM_DETECT_MOTOR_R_L`、`COMM_DETECT_MOTOR_FLUX_LINKAGE`、`COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP`、`COMM_DETECT_ENCODER`、`COMM_DETECT_HALL_FOC`、`COMM_DETECT_APPLY_ALL_FOC`（枚举在 `datatypes.h`，处理在 `comm/commands.c`） |
| 一次做完 R/L/ψ 并写回 | `conf_general_detect_apply_all_foc()` |
| 只探测霍尔或 AS5047 并改 `foc_sensor_mode` | `conf_general_autodetect_apply_sensors_foc()` |

`conf_general_detect_apply_all_foc()` 在测量前把 `foc_sensor_mode` **写成** `FOC_SENSOR_MODE_SENSORLESS`。它不会根据检测结果改成 HFI 或霍尔。霍尔/编码器模式是另一条命令。
