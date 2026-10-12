# SCS215 六关节项目交接说明

交接日期：2026-09-28

## 项目内容

本项目包含 SCS215 六舵机总线控制、单关节校准、总线扫描和只读位置监视等 Python 脚本。串口默认按 `COM5`、波特率 `1,000,000` 配置。新电脑的 COM 号可能不同，连接后先用 `python scan_bus.py --list-ports` 确认。

压缩包不包含旧电脑的 `.venv` 和 `__pycache__`；请在新电脑重新安装 Python 依赖。

## 前三节课做了什么

1. **舵机总线与 ID**：扫描总线、读取舵机型号和当前位置，为每颗舵机设置唯一 ID。改 ID 时只能单独连接那颗舵机；多颗相同 ID 会冲突。对应脚本：`scan_bus.py`、`set_id.py`。
2. **位置范围与安全校准**：逐关节记录装配后的安全最小/最大位置，并保存在 `joint_ranges.json`。对应脚本：`calibrate_joint.py`。
3. **比例位置控制**：将 `0~1` 比例映射到每个舵机的已校准范围，并同步下发位置目标。课堂文件 `step3_control.py` 示范 ID 1、2 两个舵机；`six_joint_control.py` 是扩展到六个关节的版本。

目前六关节脚本还包含预设姿态保存、快捷键切换和顺序动作；这些属于后续扩展，不影响前三节的基础练习。

## 新电脑环境

准备 Windows 电脑、Python 3、VS Code（可选）和 SCS215 串口总线控制板。项目不依赖 LeRobot；Python 侧只需要 `pyserial`。舵机需由符合规格的外部电源供电，不要依赖 USB 口给六颗舵机供电。

在解压后的项目目录打开 PowerShell：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install pyserial
python scan_bus.py --list-ports
```

如果 PowerShell 阻止激活虚拟环境，也可以直接运行 `python` 命令，不激活环境。确认舵机控制板的实际串口号后，将以下示例中的 `COM5` 换成实际端口。

## 配置文件

- `joint_ranges.json` 保存每个关节实测的校准范围，是比例控制的依据。
- 连接后先扫描总线，再确认所有关节当前位置处于各自校准范围内。
- 串口端口号以新电脑实际识别到的 USB 串口为准；默认示例为 `COM5`。

## 建议操作顺序

### 1. 确认串口和舵机

```powershell
python scan_bus.py --list-ports
python scan_bus.py --port COM5
```

确认端口是 USB 串口控制板，且总线上实际存在 ID 1~6。

### 2. 校准安全范围

每次校准一个 ID。手动引导示例：

```powershell
python calibrate_joint.py --id 1 --port COM5 --manual
```

按提示输入 `YES` 后，脚本只关闭该舵机扭矩。托住机械臂，轻柔移动到两个安全端点，分别输入 `a` 和 `b`；确认提示保存后输入 `q` 退出。对其他舵机逐一重复并再次检查 `joint_ranges.json`。

重要：手动模式下舵机可能松脱或下垂；必须持续支撑该关节。退出后该舵机仍保持扭矩关闭。不要强扭、撞击机械硬限位或夹住线缆。校准范围应是机构实际安全活动区间，并在碰撞位置前留有余量。

### 3. 复查当前位置

```powershell
python scan_bus.py --port COM5
```

控制器启动时会检查所选关节当前位置是否在校准范围内；若不在范围内会拒绝使能。

### 4. 比例控制

```powershell
python six_joint_control.py --port COM5
```

控制界面按 `m` 后，输入一个 `0~1` 数值并回车，会对所有所选关节应用同一比例；也可以输入六个比例，按 ID 顺序分别控制。`0` 对应各自安全范围最小端，`1` 对应最大端。`--dry-run` 可先只预览目标，不连接舵机。

作业演示也可用一次性命令直接控制，例如 `python six_joint_control.py --port COM5 --ratios 0.5`；先用 `python six_joint_control.py --dry-run --ratios 0.5` 预览目标。六个独立比例可写为 `--ratios 0.2 0.35 0.5 0.65 0.8 0.4`。作业原理和拍摄清单见 `作业说明.md`。

运行 `python six_joint_control.py --port COM5 --continuous` 可进入逐行比例输入模式；按 ID 1~6 输入六个比例，输入 `q` 退出。先用 `--dry-run --ratios 0.5` 检查目标。时长和速度可以通过 `--duration`、`--speed` 调整。降低速度不等于降低静态保持扭矩。

控制会使能所选舵机扭矩。启动前支撑机械臂、清理运动空间，并随时准备断电。

## 主要脚本

- `six_joint_control.py`：六关节比例、预设姿态控制。
- `calibrate_joint.py`：逐关节点动或手动引导标定。
- `scan_bus.py`：扫描 ID、内部限位和当前位置。
- `monitor_positions.py`：只读连续位置监视，软件展开 10-bit 计数器回绕。
- `set_range.py`：设置舵机内部 EEPROM 限位；只有确认需要时才运行。
- 其余 `pid_*`、`read_servo.py`、`write_servo.py`、`set_id.py` 和 `step3_control.py` 为调试/教学脚本；运行前确认其端口、ID 和运动风险。
