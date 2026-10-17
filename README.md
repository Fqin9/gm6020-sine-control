# GM6020 正弦速度与位置控制

基于 STM32F407 开发板 C 型，以 1 kHz 运行位置 P 外环和速度 PI 内环，实现正弦速度、位置跟踪。本轮候选仅提高速度比例增益，D 默认关闭。支持 KEY 启停、看门狗、反馈超时与故障停止。

默认使用 CAN1（1 Mbps）、GM6020 ID 1、电流控制模式。当前候选速度 Kp=260、Ki=120、Kd=0，位置 Kp=6；参数见 [ControlConfig.h](C_board_src/ControlConfig.h)。角度前馈针对当前电机标定，更换电机可在配置中关闭。

## 运行方法

烧录后运行程序，上电默认停止。正常供电、反馈在线且故障为零后：

1. 短按并释放 KEY：运行速度正弦，幅值 3 rad/s、周期 8 秒，16 秒后停止。
2. 再短按 KEY：运行位置正弦，以启动位置为中心、幅值 1 rad、周期 8 秒，16 秒后停止。
3. 运行中按 KEY 立即停止；长按约 1 秒停止并尝试清故障。故障恢复后不会自动启动。

在 Cortex Live Watch 查看 `g_assessment_status`，其中模式 4 为速度、5 为位置；`rms_error` 为累计 RMS 误差。故障与发送错误分别查看 `g_control_status.output_fault`、`g_control_status.can_tx_errors`。

## 实机结果

2026-10-17，未加 D 的固件提交 `94abfb1`（Kp220/Ki120），本机空载，两项各完整运行 16 秒、16000 个样本，无中断、CAN 发送错误或输出饱和。下表为上一组较好参数的实测，当前 Kp260 候选尚未实机运行。

| 项目 | RMS 误差 | 最大绝对误差 |
| --- | --- | --- |
| 速度 | 0.607595 rad/s | 2.001991 rad/s |
| 位置 | 0.037457 rad（2.15°） | 0.099530 rad（5.70°） |

看门狗已启用，仅在主循环和 1 kHz 控制任务持续推进时定期喂狗。

## 本轮 P 调整（待实测）

滤波 D 试验（提交 `e169423`，P220/I120/D0.5）已完成速度项：RMS为0.632172 rad/s、最大误差2.236091 rad/s，分别比上表速度结果增加约4.05%和11.69%。虽无故障或饱和，本次没有改善，关闭D；该D版本未运行位置项。

相对较好PI版本，本轮只将Kp220提高到260，Ki120、位置P6、前馈、滤波、目标曲线和总限幅保持原值。候选已编译并测试，需重新烧录、完成相同两项16秒曲线后，才能比较原始反馈的RMS和最大误差。每组目前各一次运行、初始角度不同，不能保证不同条件下有相同效果。上一组较好源码与固件保留在提交 `94abfb1` 和本地 captures/baseline_before_filtered_d。

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
