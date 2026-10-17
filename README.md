# GM6020 正弦速度与位置控制

基于 STM32F407 开发板 C 型，以 1 kHz 运行位置 P 外环和带测量微分滤波的速度 PID 内环，实现正弦速度、位置跟踪。支持 KEY 启停、看门狗、反馈超时与故障停止。

默认使用 CAN1（1 Mbps）、GM6020 ID 1、电流控制模式。当前候选速度 Kp=210、Ki=120、Kd=0.8，D滤波15 ms，速度反馈滤波系数0.5，位置 Kp=7，摩擦前馈340；参数见 [ControlConfig.h](C_board_src/ControlConfig.h)。角度前馈针对当前电机标定，更换电机可在配置中关闭。

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

## 本轮D项滤波调整（待实测）

D滤波10 ms、速度P210/I120/D0.8时，已有速度项1 kHz统计RMS为0.604831 rad/s、最大误差1.899222 rad/s，20 ms抽样速度绝对峰值4.398230 rad/s。位置外环提高到P7后的20 ms抽样RMS为0.051364 rad（2.94°）、最大误差0.159894 rad（9.16°）；该位置结果不是1 kHz统计。各组起始角度不同，不把单次结果当作所有条件下的保证。

本轮只将 `CONTROL_ASSESSMENT_SPEED_D_FILTER_MS` 从10改为15 ms，尝试减轻速度差分噪声对D输出的影响。速度PID、位置P7、速度反馈滤波0.5、摩擦340、角度前馈、起动补偿、目标曲线与总限幅均保留当前设置。D项仍对实测速度微分，单独限幅150原始电流给定。

当前尚无15 ms候选的实机结果。重新烧录后，先运行相同16秒速度曲线，比较原始反馈峰值、RMS与低速停滞，再检查位置误差及滞后；滤波加重不保证实际误差下降。较好历史版本保留在提交 `94abfb1`。

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
