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
| `llm_agent.py` | 4.2 LLM Agent：ReAct loop + function calling |
| `skill_arm_tracking.md` | 选做：追踪 Skill 操作指南 |
| `shots/` | 拍照保存目录 |

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
```

> 注：provider 按环境变量自动选择（先 ZHIPU_API_KEY，再 SILICONFLOW_API_KEY，最后 OPENAI_API_KEY），见文件头注释。

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
