# Week7: LLM Agent —— 机械臂模仿动作 + LLM Agent 工具调用

基于 Week6 的视觉伺服/运动控制能力，把机械臂的感知与控制封装为 **Tools**，用**外部 LLM API**
（智谱 GLM-4.6V-Flash / 硅基流动 Qwen3.5-4B，均免费）完成两件事：

1. **4.1 让机械臂模仿你的动作**：拍照 → VLM 解析手臂动作 → 输出 6 关节参数 → 机械臂模仿
2. **4.2 LLM Agent function calling**：机械臂能力作为 Tools，LLM 自主决策调用
3. **选做**：机械臂追踪 Skill 操作指南

## 文件

| 文件 | 说明 |
|---|---|
| `arm_tools.py` | 机械臂工具层（7 个工具：get_robot_state / move_to_joints / move_to_pose / detect_object / start_tracking / stop_tracking / capture_frame）+ OpenAI tools schema |
| `imitate_pipeline.py` | 4.1 pipeline：拍照 → VLM → 关节 → 机械臂（任务一~四） |
| `sequence_pipeline.py` | 4.1 任务五：文本命令 → 机械臂连贯动作（DeepSeek 文本模型解析 + 动作库逐帧执行） |
| `llm_agent.py` | 4.2 LLM Agent：ReAct loop + function calling |
| `skill_arm_tracking.md` | 选做：追踪 Skill 操作指南 |
| `raise_hand_pose.json` | 真机校准的"举手"模板（VLM 识别到举手/招手/挥手时套用） |
| `side_raise_pose.json` | 真机校准的"侧平举"模板（识别到侧平举/水平伸直时套用） |
| `shots/` | 拍照保存目录（不入 git） |

## 环境变量（API Key，勿写进代码）

```powershell
# 方式一：智谱（课程推荐免费模型 GLM-4.6V-Flash）
$env:ZHIPU_API_KEY = "你的key"

# 方式二：硅基流动（免费模型 Qwen/Qwen3.5-4B）
$env:SILICONFLOW_API_KEY = "你的key"

# 方式三：DeepSeek（已实测支持图片输入，可同时用于 4.1 VLM 与 4.2 Agent）
$env:OPENAI_API_KEY = "sk-你的key"
$env:OPENAI_BASE_URL = "https://api.deepseek.com"
$env:OPENAI_MODEL = "deepseek-chat"
```

## 运行

```powershell
$env:KMP_DUPLICATE_LIB_OK = "TRUE"   # 防 OMP Error #15

# 4.1 全流程（真机：拍照→解析→机械臂模仿，provider 由环境变量自动选择）
python lerobot_homework\week7_llm_agent\imitate_pipeline.py

# 4.1 dry-run（不连机械臂，验证 API 和流程）
python lerobot_homework\week7_llm_agent\imitate_pipeline.py --dry-run

# 4.2 LLM Agent（dry-run 仿真）
python lerobot_homework\week7_llm_agent\llm_agent.py --dry-run --task "检测黑色小球，然后追踪它"

# 4.2 真机
python lerobot_homework\week7_llm_agent\llm_agent.py --task "看看机械臂状态，把末端移到 (0.2, 0, 0.3)"

# 4.1 任务五：文本命令 → 连贯动作（真机，DeepSeek 文本模型解析"先挥手再点头"）
python lerobot_homework\week7_llm_agent\sequence_pipeline.py --command "先挥手再点头"

# 任务五 dry-run（不连机械臂，验证命令解析与动作序列）
python lerobot_homework\week7_llm_agent\sequence_pipeline.py --command "挥手" --dry-run
```

> 注：provider 按环境变量自动选择（先 ZHIPU_API_KEY，再 SILICONFLOW_API_KEY，最后 OPENAI_API_KEY），见文件头注释。

## 任务四：prompt 改进说明（向助教汇报用）

任务三 → 任务四在 prompt 中补充了：

1. **机械臂构造说明**：明确 6 自由度对应人体动作（shoulder_pan=转肩、shoulder_lift=抬臂、elbow_flex=弯肘、wrist_flex=压腕/抬手背、wrist_roll=转手腕、gripper=五指张合），让 VLM 理解"自由度↔人体关节"的对应。
2. **动作→关节参数示例表**：写入经真机校准的准确参数（举手=pan31/lift-100/elbow71、侧平举=pan23/lift-100/elbow6、叉腰、指点、比耶、握拳、下压扶物等），模型识别到类似动作时直接参考。
3. **判断原则**：先判断手臂整体朝向（抬/平/垂）和手部状态（张/握/压/朝上），再逐关节映射；并强调"水平=0°、竖直=90°"避免 90° 偏移。

额外保障（不在 prompt 内，但显著提升验收稳定性）：VLM 描述命中"举手/招手/挥手"或"侧平举/水平伸直"时，**直接套用真机保存的模板关节值**（raise_hand_pose.json / side_raise_pose.json），不再依赖模型猜测。

## 任务五：连贯动作说明（向助教汇报用）

- **命令→动作序列**：调用文本大模型（DeepSeek deepseek-chat，OpenAI 兼容）把自然语言命令（如"先挥手再点头"）解析为动作名序列 JSON；解析失败时回退关键词匹配。
- **动作库**：8 个动作（举手/侧平举/挥手/招手/点头/敬礼/叉腰/指点），每个动作由 1~5 帧关节姿态组成，逐帧降速执行。
- **验收演示**：`sequence_pipeline.py --command "先挥手再点头"` → 机械臂无人工干预连续完成挥手（5 帧摆动）+ 点头（5 帧腕部俯仰），两个不同姿态。也可组合任意动作（如"敬礼然后叉腰"）。

## 关节定义（VLM prompt 用）

| 关节 | 范围(°) | 含义 |
|---|---|---|
| shoulder_pan | -110 ~ 110 | 肩部左右旋转 |
| shoulder_lift | -100 ~ 100 | 肩部抬升 |
| elbow_flex | -97 ~ 97 | 肘部弯曲 |
| wrist_flex | -95 ~ 95 | 腕部俯仰 |
| wrist_roll | -157 ~ 163 | 腕部旋转 |
| gripper | -10 ~ 100 | 夹爪开合 |

## 安全

- 模型生成的位置参数可能不可控：运行前确保机械臂周围空旷（任务书警告）。
- 所有关节角自动裁剪到 URDF 限位；移动前建议先 `get_robot_state`。
