# Skill: 机械臂视觉追踪控制（arm-tracking）

> Week7 4.2 选做：为机械臂工具编写的最简 Skill（Markdown 操作指南，供 LLM Agent 调用）。

## 何时使用

用户要求机械臂**追踪/跟随一个物体**，或要求"看见并抓住/对准某物"时使用本 Skill。

## 调用前检查

1. 机械臂已上电、舵机在线（可先 `get_robot_state`，error 字段必须为 None）。
2. 腕部相机可用（`--camera-id 1`）。
3. 明确用户想追踪的物体描述（颜色/形状/名字）。

## 执行步骤

1. **detect_object(description)**：先用文本描述检测物体。
   - 返回 `found=false` 时：报告"未找到"，询问用户调整物体位置或换描述，**不要**强行开始追踪。
2. **start_tracking(object_id)**：用返回的 `object_id` 启动追踪。
3. 每 2 秒观察一次追踪状态（`get_robot_state` 的 `tracking` 字段）。
4. 用户说"停止/别跟了"或目标丢失超过 5 秒 → **stop_tracking()**。

## 何时必须停止

- 用户明确要求停止；
- 机械臂报错（error 非 None / 舵机过载）——先 `stop_tracking()` 再排查；
- 目标长时间丢失（>5s），避免机械臂乱动撞到人或物。

## 注意事项

- 移动前先 `get_robot_state`，确认当前姿态再动，避免幅度过大。
- 关节角度单位是度，须在限位内；超限工具会自动裁剪，但尽量由模型直接给合理值。
