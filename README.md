# GM6020 正弦速度与位置控制

STM32F407 开发板 C 型，CAN1，GM6020 ID 1，1 kHz 控制。实现自定义正弦速度跟踪、正弦位置串级控制、看门狗、显式启停、故障锁存、曲线记录和可选 VOFA+ 上传。

**状态：基线固件的速度、位置两项正弦均已在实机完整运行16秒并保存曲线，未触发故障。** 当前源码新增受限角度前馈并提高位置外环增益，属于待实机比较的优化版本。下表保留优化前的真实误差；任务未给出量化验收门槛，不将自动结束标志当作精度达标结论。

## 优化前实机结果（2026-10-17）

| 项目 | 曲线 | 平均绝对误差 | RMS误差 | 最大绝对误差 |
| --- | --- | --- | --- | --- |
| 速度 | 幅值3 rad/s、周期8秒，两个周期 | 0.795250 rad/s | 0.986526 rad/s | 3.410320 rad/s |
| 位置 | 围绕启动位置、幅值1 rad、周期8秒，两个周期 | 0.070222 rad（4.02°） | 0.098184 rad（5.63°） | 0.302906 rad（17.36°） |

两项均为16000个1kHz累计样本，`completed=1`、`aborted=0`，无CAN发送错误和输出故障。另分别保存801点约50Hz RAM日志用于曲线导出；抽样统计与1kHz累计值会有小幅差异。位置抽样目标/反馈相关系数约0.991，仅用于描述趋势，不能替代误差指标。原始实测文件保留在本地captures，未上传到源码仓库。

## 直接演示

上电不自动转动。电机和开发板正常供电，确认反馈在线、`g_control_status.output_fault=0` 后：

1. 短按并释放 **KEY**：速度正弦，峰值 3 rad/s、周期 8 秒、两个周期，共16秒后停止。
2. 等待停止，再短按一次 **KEY**：位置正弦，以本次启动时的累计角度为中心、幅值1 rad、周期8秒、两个周期，共16秒后停止。
3. 再次短按从速度模式开始。运行中按 KEY 会停止，松开不重启；中途停止不自动切换到下一项。
4. 长按 KEY 约1秒：停止并尝试清故障，松开不启动。故障清除需要新鲜反馈且旧CAN发送结束。

当前只需现有 DAPLink 和 KEY 即可运行。SWD 并不提供串口，VOFA+ 是可选输出路径。

速度曲线：`ω_ref = 3 sin(2πt/8)` rad/s。
位置曲线：`θ_ref = θ_start + sin(2πt/8)` rad；位置外环输出速度修正，并加目标轨迹的解析速度前馈，送速度内环。

上表实测基线参数：速度Kp=180、Ki=180，位置Kp=4，平滑摩擦前馈420原始值、连续低速至少100ms后缓慢增加的起动补偿最大200，电流给定限幅±2500（指令比例约±0.458 A）。RPM反馈滤波系数0.5，时间尺度约1～2ms。

首轮速度参数Kp300、Ki600、滤波系数0.1时，1kHz RMS误差约1.284 rad/s。调整到上述基线后约0.987 rad/s，下降约23%；改善并不等于消除了速度波动。

### 本次优化

位置实测的主要正弦分量约滞后0.10秒，位置外环Kp由4提高到6，继续使用目标速度前馈。速度PI及滤波、目标幅值/周期、16秒时限、电流总限幅和所有故障停止条件保持原值。

三段已有曲线中的给定存在相似的角度相关项，新增按实际累计角度计算的前馈：`-25 sin(10θ) - 110 cos(10θ) + 25 sin(20θ) - 25 cos(20θ)`，单独限幅±160原始值。接近零速参考时平滑淡出，不随转向翻转；前馈与PI合计后仍受总电流限幅和抗积分饱和约束。

这是当前这台电机的经验补偿，离线拟合不能证明实机误差已经降低；更换电机时可把`CONTROL_ASSESSMENT_ANGLE_FF_ENABLED`设为0。重新运行同样两项曲线后，用原始反馈统计的`rms_error`与上表比较，目标曲线与误差计算没有修改。测试覆盖前馈周期性、正反方向、零速淡出、非有限输入、限幅及禁用配置。

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

### 推荐：只读一键导出

当前工作目录的VS Code已添加任务“导出并打开考核曲线（只读）”。演示自动停止后，保持调试连接，按Ctrl+Shift+P，搜索“任务: 运行任务 / Tasks: Run Task”，选择该任务。它会保存数据并用系统默认程序打开SVG；也可在captures目录中右键SVG，选择浏览器打开。

从源码仓库下载后，可以在工程终端运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\capture_assessment.ps1 -Open
```

工具只连接当前OpenOCD的本机TCL端口（默认50001），用与调试会话一致的ELF符号查找RAM地址，检查演示已经停止后读取日志；不会暂停、复位、烧录或启动电机。需要保持VS Code DAPLink/OpenOCD会话连接，不能先按Shift+F5结束调试。

默认ELF为`$env:TEMP/stm32-first-assessment-debug/C_project.elf`，与现有调试配置相同；工具会从PATH或当前用户Arm/bin查找nm，从当前用户Codex运行时或PATH查找Python3。不同环境可显式传入`-ElfPath`、`-NmPath`、`-PythonPath`、`-TclPort`。不依赖固定的RAM地址，ELF必须与正在调试的固件一致。

文件自动保存在`captures/speed_时间.*`或`captures/position_时间.*`：`.bin`为原始RAM记录，`.csv`为数值，`.svg`为目标/实际曲线，`.json`为误差统计。读取前后会检查运行状态和日志头，防止导出过程中恰好开始下一项导致混合数据。请在导出完成后再按KEY启动下一项。

### 手动：GDB导出后转换

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
