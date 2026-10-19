# SO-ARM101 LeRobot 视觉闭环控制

基于 [HuggingFace LeRobot](https://github.com/huggingface/lerobot) 的六自由度机械臂（SO-ARM101，6×SCS215 舵机）课程作业：**基于视觉的闭环控制**——摄像头实时检测黑色小球，逆运动学（IK）解算目标位姿，闭环控制机械臂朝小球移动并对准。

## 目录结构

```
lerobot_homework/
├── week1_2_servo_basics/       # 第 1-2 周：双舵机控制入门
│   ├── read_servo.py / write_servo.py / set_id.py
│   ├── two_servo_control.py / sweep_servo_5s.py
│   └── 课程报告_双舵机控制.md
├── week4_vision_closed_loop/   # 第 4 周：2.2 基于视觉的闭环控制
│   ├── vision_closed_loop.py   # 视觉闭环主程序（平滑/三维追踪/方向自适应/容错）
│   ├── report_2_2_视觉闭环.md   # 2.2 作业报告
│   └── start_vision.bat        # 一键启动
├── week6_visual_servoing/      # 第 6 周：Visual Servoing 课堂练习
│   ├── camera_calibration.py   # 练习1：相机内参标定（棋盘格 15+ 张，误差~1.0）
│   ├── handeye_calibration.py  # 练习2：手眼标定（12+ 姿态，误差<5cm）
│   ├── servo_aruco.py          # 练习3：视觉伺服（ArUco 简单版，1cm 步进）
│   ├── make_printables.py      # 生成棋盘格/ArUco 打印图
│   └── printables/             # 打印图（棋盘格 9×6@25mm、ArUco 4x4_250）
├── week7_llm_agent/            # 第 7 周：LLM Agent
│   ├── arm_tools.py            # 机械臂工具层（7 个工具 + OpenAI schema）
│   ├── imitate_pipeline.py     # 4.1 拍照→VLM→关节→机械臂模仿
│   ├── llm_agent.py            # 4.2 LLM Agent ReAct + function calling
│   ├── skill_arm_tracking.md   # 选做：追踪 Skill
│   └── README.md               # Week7 使用说明（API Key/运行方式）
└── tools/                      # 通用工具与 SCS215 适配
    ├── scs215_arm.py           # 自研 SCS215 总线库（大端）
    ├── scs215_adapter.patch    # LeRobot 官方 Feetech 驱动适配补丁
    ├── record_replay.py        # 示教记录 + 重放
    ├── save_home.py / move_test.py / probe.py / ping_ids.py ...
    └── SO101/                  # 运动学 URDF
```

## 硬件

| 部件 | 说明 |
|---|---|
| SO-ARM101 机械臂 | 6 自由度 + 夹爪，6×SCS215 舵机（ID 1-6） |
| 控制板 | Waveshare Bus Servo Adapter (A)，**跳线帽必须拨到 B（USB 模式）** |
| USB 相机 | 机器人相机（示例中为 index 1） |
| 电源 | SCS215 建议 6~8.4V |

## 环境

- Python 3.12（conda env：`lerobot`）
- LeRobot 0.6.2（官方源码 editable 安装，`D:\lerobot_workspace\lerobot`）
- feetech-servo-sdk、opencv-python（GUI 版）、pyserial、placo（conda-forge）

## 快速开始

```powershell
# 1. 激活环境
conda activate lerobot

# 2. 摄像头/黑球检测测试（不动机械臂）
python lerobot_homework\vision_closed_loop.py --mode camera --camera-id 1

# 3. 完整视觉闭环（黑球放机械臂前方，按 q 退出）
$env:KMP_DUPLICATE_LIB_OK="TRUE"
python lerobot_homework\vision_closed_loop.py --mode vision --camera-id 1
```

> 注意：OMP Error #15 是 torch 与 placo 的 OpenMP 冲突，需设置 `KMP_DUPLICATE_LIB_OK=TRUE`（start_vision.bat 已内置）。

## SCS215 适配说明

LeRobot 官方 Feetech 驱动默认型号为 STS3215（型号 777），SO-ARM101 使用 SCS215（型号 1315，**寄存器大端存储**、协议 1、分辨率 1024、默认 1M 波特率），需打补丁 `scs215_adapter.patch`，修改 4 个源文件：

- `motors/feetech/tables.py` — 增加 `scs215` 全表条目 + Overload_Torque 寄存器
- `motors/motors_bus.py` — 协议 1 兼容（set_half_turn_homings / record_ranges_of_motion）
- `robots/so_follower/so_follower.py` — 六电机改 `scs215`、协议 1、逐电机读位置
- `motors/feetech/feetech.py` — **关键补丁**：`scs.SCS_END = 1`（字节序，参考实现缺失）

应用补丁：在 `D:\lerobot_workspace\lerobot` 下执行 `git apply lerobot_homework\scs215_adapter.patch`。

## 已知问题

- 控制板 USB 5V 供电时舵机电压偏低（4.9~5.0V），堵转可能触发电源保护导致舵机掉线；链路丢失时先检查跳线帽（B）、电源线和 USB 接触。
- 舵机指示灯不亮、动作一次后彻底无响应：先按上一条排查供电和跳线帽，再断开总线逐颗上电单独复测，确认是哪一颗舵机或哪段线缆出问题。
- 电压抖动时偶发 `Input voltage error`（舵机指示灯闪烁 = 欠压保护），程序已容错（跳步不崩溃），建议使用外接高电流电源。

## 2026-10-12 更新：视觉闭环 v2

对 `vision_closed_loop.py` 的完整升级，解决真机联调中暴露的检测抖动、方向错乱、运动呆板等问题：

| 问题 | 解决方案 |
|---|---|
| 黑球检测每帧跳变（dx 在 ±400px 乱跳） | 半径合理性过滤（25~250px）+ 一阶低通平滑（--smooth 0.5）+ 检测丢失保持 |
| 机械臂只在水平面平移（2/3 关节不动） | **三维追踪**：dx→左右、**球半径变化→前后**（变大=太近后退）、dy→上下 |
| 上下/左右方向反 | **方向全自动自适应**：某轴连续 2 步偏差不降反升即自动翻转该轴（终端打印 `[auto] x/y 轴方向自动翻转`），无需手动参数 |
| 腕部 4/5/6 跟着乱动 | **腕部锁定**：IK 解算后 4/5/6 锁回当前值，只让 1/2/3（肩/肘）追踪移动 |
| 球丢失后机械臂停在半路 | **丢球回位**：丢失时自动回到最后一次检测到球时的目标位，原地等待 |
| 电压抖动导致程序崩溃 | 下发容错：单次读位置失败跳过该步继续跑 |
| 每次启动回固定初始位 | `save_home.py` 固化当前位姿；删除 `home_position.json` 即"怎么摆就从哪里开始" |

常用新参数：`--smooth`（平滑强度）、`--gain-dr`（前后灵敏度）、`--max-step`（每步末端移动量，默认 0.02m）。

## 2026-10-12 更新：Week6 手眼标定

`handeye_calibration.py` 完成课程任务 2（手眼标定），支持三种采集模式：

| 模式 | 用法 | 说明 |
|---|---|---|
| **自动旋转（默认）** | `--camera-id 1` | 发送关节角自动旋转，**识别到棋盘格后只在附近 ±4° 小幅转动**（`--jitter` 可调），采集 14 个姿态 |
| 手动 | `--manual` | 关力矩手掰机械臂 + 按空格记录 |
| 重算 | `--recompute` | 从已存 `poses.json` + 照片重算，无需连机械臂（采集数据崩溃不丢） |

关键改动：
- **手写 Tsai-Lenz（AX=XB）**：OpenCV 5.0 移除了 `cv2.calibrateHandEye`，用 numpy + scipy 自实现旋转（skew(rA+rB)·r=rB−rA 最小二乘）与平移（(R_A−I)·t_X=R_X·t_B−t_A），输出 E_T_C
- **启动探测带实时预览窗口**：摆好棋盘格自动识别规格（内参优先 → 7×5 → 9×6 → 6×4 → 10×7）
- **降速防过载**：连接后写 `Running_Time`（`--run-time` 默认 800ms/步），慢速移动避免 USB 供电瞬时电流触发 Overload error
- **读位置容错**：过载报错自动等待重试，不崩程序
- 结果保存 `lerobot_homework/calib_data/handeye_result.json`（E_T_C + 回代误差，目标 <5cm）

运行（棋盘格完整入镜后自动开始）：
```powershell
$env:KMP_DUPLICATE_LIB_OK="TRUE"
python lerobot_homework\week6_visual_servoing\handeye_calibration.py --camera-id 1
```
