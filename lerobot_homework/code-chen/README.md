# SCS215 六自由度机械臂控制程序（24 组）

用六颗 SCS215 串行总线舵机搭的六自由度机械臂，Windows + Python 通过 USB 转串口板控制。

包含三块功能：

1. **多关节连续控制**：预设姿态切换、动作序列、可调切换时间与速度。
2. **逆运动学（IK）控制**：用键盘或文字命令让末端执行器沿直线移动。
3. **语音控制**：语音识别成文字，再解析成机械臂动作。

## 一、硬件与软件要求

- 六颗 SCS215 总线舵机 + USB 转串口控制板
- 舵机要接**外部电源**，不要靠 USB 口给六颗舵机供电
- Windows 电脑、Python 3

核心功能只需要一个第三方库：`pyserial`。

## 二、快速开始

在项目目录打开 PowerShell，依次执行：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python scan_bus.py --list-ports
```

`--list-ports` 会列出你电脑上识别到的串口。**记下控制板对应那个口**（比如 `COM7`），后面所有命令把示例里的 `COM5` 换成它。

确认能连上舵机：

```powershell
python scan_bus.py --port COM5
```

六关节比例控制：

```powershell
python six_joint_control.py --port COM5
```

按键：`1~9` 切换姿态，`a` 执行动作序列，`t` 设置切换时间，`v` 设置速度，`q` 退出。

逆运动学控制：

```powershell
python ik_control.py --port COM5
python ik_control.py --port COM5 --mode keyboard
```

语音控制：

```powershell
python voice_control.py --port COM5
python voice_control.py --port COM5 --mode text
```

## 三、换电脑 / 换人必须先改端口

代码里的默认串口号是 **COM5**，这是原作者的电脑上的编号。**别人的电脑上大概率不是 COM5**，直接运行会报“无法打开串口”。

正确做法有两种，任选一种：

1. **推荐**：不改代码，运行时加参数。
   `python six_joint_control.py --port COM7`
2. 改代码默认值：把下表中的 `PORT = "COM5"` 改成自己的串口号。

各文件的端口位置：

| 文件 | 端口所在行 | 支持 `--port` 参数 |
| --- | --- | --- |
| `six_joint_control.py` | 第 12 行 | 是 |
| `ik_control.py` | 第 26 行 | 是 |
| `voice_control.py` | 无（沿用 `ik_control.py` 的默认值） | 是 |
| `scan_bus.py` | 第 17 行 | 是 |
| `calibrate_joint.py` | 第 12 行 | 是 |
| `monitor_positions.py` | 第 10 行 | 是 |
| `set_id.py` | 第 6 行 | 是 |
| `set_range.py` | 第 8 行 | 否（改代码） |
| `pid_tune.py` | 第 34 行 | 否（改代码） |
| `pid_curve.py` | 第 31 行 | 否（改代码） |
| `step3_control.py` | 第 15 行 | 否（改代码） |
| `write_servo.py` | 第 4 行 | 否（改代码） |
| `read_servo.py` | 第 4 行 | 否（改代码） |

波特率统一是 `1_000_000`（1M），SCS 系列默认值，不要改。

另外，`docs/` 里的说明文档和报告中的示例命令写的都是 `COM5`，照着跑同样要换成自己的端口。

## 四、文件说明

### 主功能

| 文件 | 作用 |
| --- | --- |
| `six_joint_control.py` | 六关节比例控制，预设姿态切换、动作序列、时间与速度设置 |
| `ik_control.py` | 逆运动学控制，键盘 / 文字模式，也提供 `--move`、`--goto` 等子命令 |
| `voice_control.py` | 语音或文字指令控制，内部调用 `ik_control.py` |

### 工具脚本

| 文件 | 作用 |
| --- | --- |
| `scan_bus.py` | 扫描总线上的舵机 ID，列出串口 |
| `set_id.py` | 给单颗舵机改 ID（必须只接这一颗） |
| `set_range.py` | 设置舵机内部位置范围寄存器 |
| `calibrate_joint.py` | 逐关节记录装配后的安全最小 / 最大位置 |
| `monitor_positions.py` | 只读监视各关节当前位置 |
| `read_servo.py` / `write_servo.py` | 最小示例，手写控制帧读写单个舵机 |
| `pid_tune.py` / `pid_curve.py` | PID 参数调试与曲线绘制（需要 `matplotlib`） |
| `step3_control.py` | 课堂示例：只控制 ID 1、2 两颗舵机 |

### 配置与文档

| 文件 | 说明 |
| --- | --- |
| `joint_ranges.json` | 每个关节实测的安全范围 `[min_step, max_step]`，比例控制的依据 |
| `joint_poses.json` | 预设姿态，记录六个舵机的目标步数 |
| `docs/报告.md` | 第 4 周实验报告 |
| `docs/逆运动学控制说明.md` | IK 原理与用法 |
| `docs/语音控制说明.md` | 语音控制的依赖、模型与 API 配置 |
| `docs/HANDOFF.md` | 项目交接说明、上手顺序 |

## 五、语音控制的额外依赖

语音控制是可选项，核心功能不需要。它需要额外的语音识别库：

```powershell
python -m pip install -r requirements-voice.txt
```

如果用大模型解析自然语言，需要设置 API Key 环境变量，**不要把 Key 写进代码或提交到仓库**：

```powershell
$env:DEEPSEEK_API_KEY = "你的 Key"
python voice_control.py --port COM5 --llm
```

也可以用任意 OpenAI 兼容接口，设置 `OPENAI_API_KEY` 和 `OPENAI_BASE_URL`。细节见 `docs/语音控制说明.md`。

## 六、安全提醒

- 校准和安全范围操作前先托住机械臂，避免舵机突然运动或掉落。
- 程序退出后舵机扭矩可能仍保持使能，注意支撑机械臂。
- 别用 USB 口给六颗舵机供电。
