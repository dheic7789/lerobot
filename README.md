# SO-ARM101 LeRobot 视觉闭环控制

基于 [HuggingFace LeRobot](https://github.com/huggingface/lerobot) 的六自由度机械臂（SO-ARM101，6×SCS215 舵机）课程作业：**基于视觉的闭环控制**——摄像头实时检测黑色小球，逆运动学（IK）解算目标位姿，闭环控制机械臂朝小球移动并对准。

## 目录结构

```
lerobot_homework/
├── vision_closed_loop.py        # 2.2 主程序：视觉闭环（vision / camera / check 三种模式）
├── start_vision.bat             # 一键启动脚本（含 OMP 冲突修复环境变量）
├── report_2_2_视觉闭环.md        # 2.2 作业报告
├── scs215_arm.py                # 自研 SCS215 总线库（大端字节序：scan / set-id / calibrate / control）
├── scs215_adapter.patch         # LeRobot 官方 Feetech 驱动适配 SCS215 的补丁（4 个源文件）
├── make_lerobot_calib.py        # 自研校准 → LeRobot 校准文件转换
├── burn_calib.py                # 校准烧写进舵机 EEPROM
├── lerobot_bus_test.py          # 连接/读数测试
├── lerobot_write_test.py        # 写位置测试
├── lerobot_robot_test.py        # 机器人级连接/校准/观测测试
├── cam_vs_com3_test.py          # 摄像头 vs 串口干扰诊断
├── move_test.py                 # 真机连读连写连动测试
├── save_home.py                 # 把当前舵机位姿保存为固定初始位
├── home_position.json           # 固定初始位（可选；删除后以当前位姿为初始位）
├── SO101/                       # 运动学 URDF（so101_kinematics.urdf 为 IK 实际使用版）
├── arm_calibration.json         # 六关节原始编码校准范围
└── 早期测试脚本                  # read_servo / write_servo / set_id / two_servo_control / sweep_servo_5s 等
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
