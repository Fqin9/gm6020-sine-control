# GM6020 正弦速度与位置控制

基于 STM32F407 开发板 C 型，以 1 kHz 运行位置 P 外环和速度 PI 内环，实现正弦速度、位置跟踪。支持 KEY 启停、看门狗、反馈超时与故障停止。

默认使用 CAN1（1 Mbps）、GM6020 ID 1、电流控制模式。当前候选速度 Kp=220、Ki=120、Kd=0，位置 Kp=6，摩擦前馈340；参数见 [ControlConfig.h](C_board_src/ControlConfig.h)。角度前馈针对当前电机标定，更换电机可在配置中关闭。

## 运行方法

烧录后运行程序，上电默认停止。正常供电、反馈在线且故障为零后：

1. 短按并释放 KEY：运行速度正弦，幅值 3 rad/s、周期 8 秒，16 秒后停止。
2. 再短按 KEY：运行位置正弦，以启动位置为中心、幅值 1 rad、周期 8 秒，16 秒后停止。
3. 运行中按 KEY 立即停止；长按约 1 秒停止并尝试清故障。故障恢复后不会自动启动。

在 Cortex Live Watch 查看 `g_assessment_status`，其中模式 4 为速度、5 为位置；`rms_error` 为累计 RMS 误差。故障与发送错误分别查看 `g_control_status.output_fault`、`g_control_status.can_tx_errors`。

## 实机结果

2026-10-17，未加 D 的固件提交 `94abfb1`（Kp220/Ki120、摩擦前馈420），本机空载，两项各完整运行 16 秒、16000 个样本，无中断、CAN 发送错误或输出饱和。下表为该组历史实测，不代表当前候选的效果。

| 项目 | RMS 误差 | 最大绝对误差 |
| --- | --- | --- |
| 速度 | 0.607595 rad/s | 2.001991 rad/s |
| 位置 | 0.037457 rad（2.15°） | 0.099530 rad（5.70°） |

看门狗已启用，仅在主循环和 1 kHz 控制任务持续推进时定期喂狗。

## 本轮起动补偿调整（待实测）

滤波D0.5试验速度RMS为0.632172 rad/s、最大误差2.236091 rad/s；P260试验为0.661287 rad/s、4.477377 rad/s，均未优于上表。当前恢复P220、D0。

用户将摩擦前馈调整为340后，曲线仍存在低速停滞。本轮相对该配置只将起动补偿增长量从0.5改为1.0 raw/ms（`CONTROL_ASSESSMENT_BREAKAWAY_RAMP_RAW_PER_MS`）。连续低速等待100 ms、补偿上限200、退出后的衰减系数0.99保持原值；持续满足条件时，约300 ms达到补偿上限，原来约500 ms。PI、位置P、滤波0.5、角度前馈、曲线与总限幅均保持当前设置。

候选需重新烧录并运行相同16秒速度曲线，比较低速停滞时间、实际速度峰值、RMS及最大误差；速度改善后再运行位置项。当前尚无新候选的实机结果。各组初始角度不同，单次比较不能保证其他条件下有相同效果；较好历史版本保留在提交 `94abfb1`。

## 构建与测试

构建需要 ARM GNU 工具链、CMake、Ninja；主机测试另需 Python 3 和 g++。

```sh
cmake --preset Debug
cmake --build --preset Debug
python tests/run_tests.py
```

固件产物为 `build/Debug/C_project.elf`，可通过 DAPLink/OpenOCD 烧录调试。

## 主要代码

- [AssessmentController.cpp](C_board_src/AssessmentController.cpp)：正弦轨迹、位置 P、速度 PI、测量速度的滤波 D 和前馈。
- [ControlTask.cpp](C_board_src/ControlTask.cpp)：控制调度、CAN、启停、保护与曲线记录。
- [KeyControl.cpp](C_board_src/KeyControl.cpp)：KEY 按键入口。
- [export_assessment.py](tools/export_assessment.py)：将实际 RAM 日志转换为 CSV、SVG 和误差统计；下一次启动会覆盖板端日志。
