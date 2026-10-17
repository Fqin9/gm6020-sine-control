# GM6020 正弦速度与位置控制

基于 STM32F407 开发板 C 型，以 1 kHz 运行位置 P 外环和速度 PI 内环，实现正弦速度、位置跟踪。最新候选增加实际速度的滤波 D 项。支持 KEY 启停、看门狗、反馈超时与故障停止。

默认使用 CAN1（1 Mbps）、GM6020 ID 1、电流控制模式。当前速度 Kp=220、Ki=120，位置 Kp=6；参数见 [ControlConfig.h](C_board_src/ControlConfig.h)。角度前馈针对当前电机标定，更换电机可在配置中关闭。

## 运行方法

烧录后运行程序，上电默认停止。正常供电、反馈在线且故障为零后：

1. 短按并释放 KEY：运行速度正弦，幅值 3 rad/s、周期 8 秒，16 秒后停止。
2. 再短按 KEY：运行位置正弦，以启动位置为中心、幅值 1 rad、周期 8 秒，16 秒后停止。
3. 运行中按 KEY 立即停止；长按约 1 秒停止并尝试清故障。故障恢复后不会自动启动。

在 Cortex Live Watch 查看 `g_assessment_status`，其中模式 4 为速度、5 为位置；`rms_error` 为累计 RMS 误差。故障与发送错误分别查看 `g_control_status.output_fault`、`g_control_status.can_tx_errors`。

## 实机结果

2026-10-17，未加 D 的固件提交 `94abfb1`，本机空载，两项各完整运行 16 秒、16000 个样本，无中断、CAN 发送错误或输出饱和。下表是该版本的历史实测，新增 D 后的效果尚待确认。

| 项目 | RMS 误差 | 最大绝对误差 |
| --- | --- | --- |
| 速度 | 0.607595 rad/s | 2.001991 rad/s |
| 位置 | 0.037457 rad（2.15°） | 0.099530 rad（5.70°） |

看门狗已启用，仅在主循环和 1 kHz 控制任务持续推进时定期喂狗。

## 本轮滤波 D 调整（待实测）

P=220、I=120、位置 P=6 和前馈保持原值，新增 Kd=0.5；D 支路使用 10ms 一阶滤波，单独限幅±150 原始给定，总给定仍限幅±2500。D 对实际速度求导，目标变化不直接产生微分冲击；启动时清除历史。该[PI-D结构](https://www.mathworks.com/help/slcontrol/ug/create-i-pd-and-pi-d-controllers.html)与微分滤波用于改善抗扰，不能据此保证当前电机的效果。

重新烧录并完成相同两项16秒曲线后，与上表的 RMS 和最大误差比较；统计仍使用原始反馈。Kd 设为0可禁用 D，上一实测源码及固件保留在提交 `94abfb1`，本地另有 captures/baseline_before_filtered_d 备份。

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
