> 本文档由飞书知识库自动导出（Lerobot知识库），与 https://h106ar60fgq.feishu.cn/docx/KGGzdY2Vko7j0DxndC2cAWBKnDe 同步。

# Lerobot知识库

# 课程内容

第一周 官网：

<readonly-block href="https://eai-course.github.io/" type="iframe"></readonly-block>

第二周 vibe coding： <cite doc-id="R3i5wIZq4iWKXlkU7xdcTsMRnfb" file-type="wiki" title="EmbodiedAI Week 2: Vibe Coding" type="doc"></cite>

第三周 舵机与运动控制：<cite doc-id="Er3nwu6JNiZ5fqklfzzcKkhGn2g" file-type="wiki" title="EmbodiedAI Week 3：舵机与运动控制" type="doc"></cite>

第四周 语音与视觉控制：<cite doc-id="BJN3w4Hc2iqSRVkS6x8cSNjonSe" file-type="wiki" title="EmbodiedAI Week 4: Hackathon" type="doc"></cite>

第六周 视觉控制：<cite doc-id="D1jWwPc3PikTAFkHkTkcfgJknEf" file-type="wiki" title="EmbodiedAI Week 6: Visual Servoing" type="doc"></cite>

第七周 大语言模型助手：<cite doc-id="LVVow0AxIiPbGDk5zedcuGaLnqc" file-type="wiki" title="EmbodiedAI Week 7: LLM Agent" type="doc"></cite>

# 舵机

通信协议：

<bookmark name="FT管理平台" href="http://doc.feetech.cn/#/tiaozhunlujingft?srcType=FT-SCS-Protocol-41ad23fe8a244712ba160b93"></bookmark>

内存表：

<bookmark name="FT管理平台" href="http://doc.feetech.cn/#/tiaozhunlujingft?srcType=FT-SCS-Protocol-41ad23fe8a244712ba160b93"></bookmark>



# 本地实践记录（2026-10）

## 环境

- Python 3.12（conda env: lerobot，miniforge3）
- LeRobot 0.6.2 editable：D:\lerobot_workspace\lerobot
- 运行必须设置 KMP_DUPLICATE_LIB_OK=TRUE（torch 与 placo 的 OpenMP 冲突，否则 OMP Error #15）
- GitHub 仓库：https://github.com/dheic7789/lerobot（按周分类）

## SCS215 舵机实测要点

- 型号 1315（0x0523）：2 字节寄存器大端存储（与 STS3215 小端不同）
- 协议 1、默认 1M 波特率、分辨率 1024
- 控制板 Waveshare Bus Servo Adapter (A)：跳线帽 JA/JB 必须拨到 B（USB 模式）
- 总线串联：一根线松 → 其后整段舵机失联
- 关键寄存器：ID=5, Baud=6, Torque_Enable=40(1B), Goal_Position=42(2B), Running_Time=44(2B), Present_Position=56(2B), Present_Voltage=62(1B), Temp=63(1B)
- 电压：额定 6\~8.4V；USB 5V 带 6 个舵机会欠压（指示灯闪烁=欠压保护，报 Input voltage error）

## 已完成练习

- Week4 · 2.2 视觉闭环：vision_closed_loop.py（检测平滑、三维追踪、方向自适应、容错）+ 示教记录重放 record_replay.py
- Week6 · Visual Servoing：相机标定 camera_calibration.py、手眼标定 handeye_calibration.py、ArUco 视觉伺服 servo_aruco.py、打印图生成 make_printables.py
- Week7 · LLM Agent：机械臂模仿 imitate_pipeline.py、Agent 工具层 arm_tools.py + function calling llm_agent.py、追踪 Skill skill_arm_tracking.md

## 排障速查

- OMP Error #15 → 设置 KMP_DUPLICATE_LIB_OK=TRUE
- 舵机全丢（scan 全 Missing）→ 检查跳线帽 B、总线线接触、供电电压
- import lerobot 报命名空间包错误 → 不要在 lerobot 子目录内运行脚本
- CMD 下 cd D: 不生效 → 用 cd /d D:\
- 舵机指示灯闪烁 = 欠压保护（Input voltage error）→ 换外接高电流电源

# 执行指令（2026-10-12 按周目录）

通用前提：先 conda activate lerobot；在 D:\lerobot_workspace 下运行；每次先设置 \$env:KMP_DUPLICATE_LIB_OK=TRUE；机械臂接 USB（跳线帽 B，COM3）。

## Week 1-2 · 双舵机

```bash
cd /d D:\lerobot_workspace
$env:KMP_DUPLICATE_LIB_OK=TRUE
python lerobot_homework\week1_2_servo_basics\read_servo.py
python lerobot_homework\week1_2_servo_basics\write_servo.py
python lerobot_homework\week1_2_servo_basics\set_id.py
python lerobot_homework\week1_2_servo_basics\two_servo_control.py
python lerobot_homework\week1_2_servo_basics\sweep_servo_5s.py
```

## Week 4 · 2.2 视觉闭环

```bash
cd /d D:\lerobot_workspace
$env:KMP_DUPLICATE_LIB_OK=TRUE
python lerobot_homework\week4_vision_closed_loop\vision_closed_loop.py --mode vision --camera-id 1
python lerobot_homework\week4_vision_closed_loop\vision_closed_loop.py --mode camera --camera-id 1
```

--mode：vision=闭环追踪 / camera=仅显示画面 / check=自检；可选 --lock-shoulder 锁肩、--lock-wrist 锁腕。

## Week 6 · Visual Servoing

```bash
cd /d D:\lerobot_workspace
$env:KMP_DUPLICATE_LIB_OK=TRUE
python lerobot_homework\week6_visual_servoing\make_printables.py
python lerobot_homework\week6_visual_servoing\camera_calibration.py --camera-id 1
python lerobot_homework\week6_visual_servoing\handeye_calibration.py
python lerobot_homework\week6_visual_servoing\servo_aruco.py
```

顺序：打印棋盘格 → 相机标定（空格存图≥15 张，c 标定）→ 手眼标定（需相机内参）→ 伺服。

## tools · 通用工具

```bash
cd /d D:\lerobot_workspace
$env:KMP_DUPLICATE_LIB_OK=TRUE
python lerobot_homework\tools\ping_ids.py
python lerobot_homework\tools\scs215_arm.py --help
python lerobot_homework\tools\record_replay.py
```

## Week 7 · LLM Agent

```powershell
cd /d D:\lerobot_workspace
$env:KMP_DUPLICATE_LIB_OK=TRUE
$env:ZHIPU_API_KEY="你的key"        # 智谱：免费 GLM-4.6V-Flash（推荐）
$env:SILICONFLOW_API_KEY="你的key"  # 硅基流动：免费 Qwen/Qwen3.5-4B
$env:OPENAI_API_KEY="sk-你的key"    # DeepSeek（已实测支持图片，4.1 VLM 与 4.2 Agent 均可用）
$env:OPENAI_BASE_URL="https://api.deepseek.com"
$env:OPENAI_MODEL="deepseek-chat"
```

```powershell
python lerobot_homework\week7_llm_agent\imitate_pipeline.py             # 全流程（真机）
python lerobot_homework\week7_llm_agent\imitate_pipeline.py --dry-run   # 仿真验证（不连机械臂）
python lerobot_homework\week7_llm_agent\imitate_pipeline.py --level 2   # 增强 prompt（任务四）
```

```powershell
python lerobot_homework\week7_llm_agent\llm_agent.py --dry-run --task "检测黑色小球，然后追踪它"
python lerobot_homework\week7_llm_agent\llm_agent.py --task "看看机械臂状态，把末端移到 (0.2, 0, 0.3)"
```

说明：工具层 arm_tools.py 提供 get_robot_state / move_to_joints / move_to_pose / detect_object / start_tracking / stop_tracking；关节角自动裁剪限位；SCS215 降速防过载；dry-run 模式不连机械臂。追踪 Skill 见 skill_arm_tracking.md。

```powershell
python lerobot_homework\week7_llm_agent\sequence_pipeline.py --command "先挥手再点头"   # 真机连贯动作
python lerobot_homework\week7_llm_agent\sequence_pipeline.py --command "挥手" --dry-run   # 仿真验证
python lerobot_homework\tools\servo_angle_test.py      # 六舵机逐角度位置测试（交互/--auto N/--only N）
```

任务四 prompt 改进（向助教汇报）：①注入机械臂 6 自由度构造说明——shoulder_pan=转肩 / shoulder_lift=抬臂 / elbow_flex=弯肘 / wrist_flex=压腕·抬手背 / wrist_roll=转腕 / gripper=五指张合，让 VLM 理解自由度↔人体关节对应；②写入真机校准的动作示例表（举手=pan31/lift-100/elbow71、侧平举=pan23/lift-100/elbow6、叉腰、指点、比耶、握拳、下压扶物）；③判断原则：先判手臂整体朝向（抬/平/垂）与手部状态（张/握/压/朝上），再逐关节映射，并强调水平=0°、竖直=90°。额外保障：VLM 描述命中"举手/招手/挥手"或"侧平举/水平伸直"时直接套用真机模板 raise_hand_pose.json / side_raise_pose.json，不依赖模型猜测关节值。

任务五：sequence_pipeline.py 用文本大模型（DeepSeek deepseek-chat）把自然语言命令解析为动作名序列，动作库 actions_library.json（8 动作 21 帧：举手/侧平举/挥手/招手/点头/敬礼/叉腰/指点，数据与代码分离可直接编辑），逐帧降速执行，无人工干预。验收演示：--command "先挥手再点头" 连续完成挥手（5 帧肩摆）+ 点头（5 帧腕俯仰）两个不同姿态。