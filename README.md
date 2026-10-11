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
- 他妈的，舵机的灯都不亮。刚才动了一下之后再也不动了
