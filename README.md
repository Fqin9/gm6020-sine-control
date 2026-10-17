# GM6020 正弦速度与位置控制

STM32F407 开发板 C 型，CAN1，GM6020 ID 1，1 kHz 控制。实现自定义正弦速度跟踪、正弦位置串级控制、看门狗、显式启停、故障锁存、曲线记录和可选 VOFA+ 上传。

**状态：固件与主机逻辑测试通过。最终两项正弦曲线尚未完成实机验收，参数是候选值，仓库不包含虚构的实测曲线或已达标声明。** 此前实机已确认 CAN 在线、电流反馈及正反向小输出响应，低速恒速试验尚未稳定跟踪。

## 直接演示

上电不自动转动。电机和开发板正常供电，确认反馈在线、`g_control_status.output_fault=0` 后：

1. 短按并释放 **KEY**：速度正弦，峰值 3 rad/s、周期 8 秒、两个周期，共16秒后停止。
2. 等待停止，再短按一次 **KEY**：位置正弦，以本次启动时的累计角度为中心、幅值1 rad、周期8秒、两个周期，共16秒后停止。
3. 再次短按从速度模式开始。运行中按 KEY 会停止，松开不重启；中途停止不自动切换到下一项。
4. 长按 KEY 约1秒：停止并尝试清故障，松开不启动。故障清除需要新鲜反馈且旧CAN发送结束。

当前只需现有 DAPLink 和 KEY 即可运行。SWD 并不提供串口，VOFA+ 是可选输出路径。

速度曲线：`ω_ref = 3 sin(2πt/8)` rad/s。
位置曲线：`θ_ref = θ_start + sin(2πt/8)` rad；位置外环输出速度修正，并加目标轨迹的解析速度前馈，送速度内环。

新演示使用独立参数：速度Kp=180、Ki=180，位置Kp=4，平滑摩擦前馈420原始值、连续低速至少100ms后缓慢增加的起动补偿最大200，电流给定限幅±2500（指令比例约±0.458 A）。RPM反馈滤波系数0.5，时间尺度约1～2ms。这些值基于首轮实测快速波动调整，修改后的跟踪误差仍须从实机记录判断，自动结束不等于跟踪达标。

首轮速度实机演示已完成16秒，未触发故障：1kHz统计MAE约1.003 rad/s、RMSE约1.284 rad/s，尚未完成整定。已只读取出801点实际RAM日志，50Hz记录的RMSE约1.253 rad/s，显示快速速度波动。上述新参数用于减小该波动；新参数和位置演示尚未验收。原始实测数据保存在本地captures中，未作为已通过证明。

## 接线与运行环境

- GM6020 当前开启内部电流环，控制帧用0x1FE，反馈ID为0x205；CAN1为1 Mbps。
- 开发板和电机各按24V电源接口供电；CAN_H接CAN_H、CAN_L接CAN_L，终端电阻按实际总线配置。SWD使用SWDIO、SWCLK、GND及正确电压参考。
- KEY为PA0低有效。USART6对应外壳UART1，PG14 TX、PG9 RX，115200、8N1；串口模块为3.3V TTL，TX/RX交叉并共地。
- 使用前固定定子，保持转子和线缆活动范围；GM6020手册规定转子M3安装孔深4mm、定子M4安装孔深6mm。不要通过拆电机本体排查手转阻力。

## 观察结果

Cortex Live Watch 添加：

```text
g_assessment_status
g_control_status.output_fault
g_control_status.can_tx_errors
g_assessment_log.count
```

`mode/next_mode`：4速度正弦，5位置正弦。`running`为运行状态；`completed=1`只表示按时完成轨迹，`aborted=1`表示中断。`rms_error/mean_abs_error/max_abs_error`分别为运行中累计误差统计，速度模式单位rad/s，位置模式单位rad。

故障位可组合：1反馈超时/会话变化，2发送故障，4非有限输出，8持续超速，16位置偏差过大，32温度过高。保护停止后不自动恢复。位置模式每次显式启动重建中心，不沿用断线前的旧基准。

## VOFA+（已接独立串口时）

选择实际串口、115200、8N1、无流控，数据引擎FireWater，命令使用文本并添加换行，关闭循环发送：

```text
PLOT 1
MODE SPEED
RUN
```

等停止后使用`MODE POSITION`、`RUN`。`STOP`停止，`CLEAR`清故障，`STATUS`查询，`PLOT 0`关闭曲线上传。切换模式只在停止时生效，切换命令本身不会启动。

上传频率50Hz：`plot:目标,实际,误差,原始给定,目标轨迹速度,实际速度`。前两列随所选模式分别使用rad/s或rad。文本回复不带plot帧格式。串口收发中断工作，接收错误/溢出会停止并锁存串口故障；控制循环不等待串口发送。

## 无串口也可导出曲线

每次演示在RAM保留约50Hz记录，下一次启动会覆盖。要保存两项，须在第二项启动前导出第一项；日志不会自动写Flash。

演示自动停止后，在 Cortex Debug 调试控制台执行以下GDB命令（临时路径使用纯英文）：

```text
-exec dump binary memory C:/Users/Public/gm6020-speed.bin &g_assessment_log ((char*)&g_assessment_log)+sizeof(g_assessment_log)
```

位置演示结束使用另一个文件名`gm6020-position.bin`。若调试器要求暂停，只在确认演示已结束、输出为零后进行。文件路径由GDB所在电脑解释；可改为其他可写英文路径。

```text
python tools/export_assessment.py C:/Users/Public/gm6020-speed.bin --out captures/speed
python tools/export_assessment.py C:/Users/Public/gm6020-position.bin --out captures/position
```

输出CSV、SVG曲线与JSON误差统计，只转换实际输入日志。20ms抽样不能证明不存在更快的振荡；完整运行和故障状态另看Live Watch。

## 构建与测试

安装ARM GNU工具链、CMake、Ninja，加入PATH：

```text
cmake --preset Debug
cmake --build --preset Debug
```

产物`build/Debug/C_project.elf`。本机已有VS Code配置继续使用原工作目录的F5流程；源码仓库不包含个人绝对路径的编辑器配置。DAPLink可用OpenOCD的`interface/cmsis-dap.cfg`、`target/stm32f4x.cfg`和SWD连接。

电脑逻辑测试需要g++和Python3：

```text
python tests/run_tests.py
```

Windows也可执行`tests/run_tests.ps1`。测试覆盖CAN编解码、模式和曲线、位置跨零、限幅与积分退出饱和、日志、按键、串口、截止、故障停止、时间回绕及看门狗喂狗条件。测试中的理想反馈由测试程序注入，不是实机跟踪结果。

看门狗已在`MX_IWDG_Init`开启，64分频、Reload400；仅当前台循环和1kHz任务都推进时喂狗。硬件看门狗复位效果须在实板确认，不能以主机测试代替。

## 文件入口

- `C_board_src/ControlConfig.h`：接口配置、曲线与候选控制参数。
- `AssessmentController.cpp`：轨迹、位置外环、速度PI、滤波与起动补偿。
- `ControlTask.cpp`：定时调度、CAN、启停、故障、日志和看门狗。
- `KeyControl.cpp`、`SerialCommand.cpp`：按键与可选串口入口。
- `Core/`、`Drivers/`、`cmake/`：CubeMX生成外设和构建支持，保留原厂许可声明。

GitHub提交要求为公开源码链接。完整考核还包括实机展示两条跟踪曲线；本仓库的代码和自动测试不等同实机验收已通过。
