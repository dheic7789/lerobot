#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Week7 4.1：让机械臂模仿你的动作 —— 拍照 → VLM 解析 → 机械臂模仿 pipeline。

任务一：capture  调用电脑/腕部相机拍照保存（不含 imshow）
任务二：vlm      调用 VLM API 解析照片中手臂动作（智谱 GLM-4.6V-Flash 免费 / 硅基流动 Qwen3.5-4B 免费）
任务三：drive    把 VLM 输出的 6 关节参数发给机械臂执行
任务四：--level 2  增强 prompt（关节构造 + 动作→关节示例），更准确模仿

用法：
  # 全流程（真机）
  python imitate_pipeline.py --provider zhipu --camera-id 0
  # dry-run（不连机械臂，验证 API + 流程）
  python imitate_pipeline.py --dry-run --provider siliconflow
  # 分步：只拍照 / 只解析 / 只驱动
  python imitate_pipeline.py --step capture
  python imitate_pipeline.py --step vlm --image shots/arm_000.jpg
  python imitate_pipeline.py --step drive --joints '{"shoulder_lift": 30, "elbow_flex": 40, "gripper": 50}'

环境变量（provider 配置）：
  ZHIPU_API_KEY       智谱（默认 base https://open.bigmodel.cn/api/paas/v4/，模型 glm-4.6v-flash）
  SILICONFLOW_API_KEY 硅基流动（base https://api.siliconflow.cn/v1，模型 Qwen/Qwen3.5-4B）
  OPENAI_API_KEY / OPENAI_BASE_URL / OPENAI_MODEL  通用 OpenAI 兼容
"""
import argparse
import base64
import json
import os
import sys
import time
from pathlib import Path

import cv2

BASE_DIR = Path(__file__).resolve().parent
SHOTS_DIR = BASE_DIR / "shots"
SHOTS_DIR.mkdir(exist_ok=True)

# 关节说明（注入 VLM prompt）
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
JOINT_RANGES = {
    "shoulder_pan": "[-110, 110] 度：肩部左右旋转，负=右转，正=左转",
    "shoulder_lift": "[-100, 100] 度：肩部抬升，负=手臂下垂，正=手臂抬起",
    "elbow_flex": "[-97, 97] 度：肘部弯曲，负=伸直，正=弯曲",
    "wrist_flex": "[-95, 95] 度：腕部俯仰，负=向下，正=向上",
    "wrist_roll": "[-157, 163] 度：腕部旋转，负=逆时针，正=顺时针",
    "gripper": "[-10, 100] 度：夹爪，负/小=闭合，正/大=张开",
}

PROMPT_L1 = (
    "你是一台 6 自由度机械臂的视觉解析器。我会给你一张人类手臂动作的照片。\n"
    "请完成两件事：\n"
    "1) 用一句话描述照片中人物的手臂在做什么动作（例如：抬起手臂挥手）。\n"
    "2) 把这个动作转换为一组机械臂 6 关节控制参数（单位：度，整数）。\n"
    "机械臂每个关节的名称、含义、活动范围如下：\n"
    + "\n".join(f"- {name}: {desc}" for name, desc in JOINT_RANGES.items()) +
    "\n\n动作→关节角度的关键映射（务必严格遵守，这是最常见的错误来源）：\n"
    "- 手臂水平向前/向侧伸出（伸直）→ shoulder_lift≈0，elbow_flex≈0（大臂水平！0 度不是 90 度）\n"
    "- 手臂竖直向上举起（举手/打招呼）→ shoulder_lift≈85~90\n"
    "- 手臂自然下垂 → shoulder_lift≈-70~-80\n"
    "- 肘部弯曲、小臂折起 → elbow_flex≈30~90；手臂完全伸直 → elbow_flex≈0\n"
    "- 手掌张开/比耶/招手 → gripper≈70~90；握拳/握持 → gripper≈-5\n"
    "把照片中手臂相对**水平线**的抬起角度映射为 shoulder_lift：水平伸出=0°，举过头顶≈90°，自然下垂≈-75°。\n\n"
    "只输出 JSON（不要解释、不要 markdown），格式：\n"
    '{"description": "动作描述", "joints": {"shoulder_pan": 0, "shoulder_lift": 30, '
    '"elbow_flex": 40, "wrist_flex": 0, "wrist_roll": 0, "gripper": 50}}\n'
    "所有关节值必须在上述范围内，尽量选择能表达照片动作的合理姿态；gripper 默认 50（半开）。"
)

PROMPT_L2 = PROMPT_L1 + (
    "\n\n补充说明（帮助你更准确模仿）：\n"
    "- 机械臂竖直向上伸直时：shoulder_lift≈90, elbow_flex≈0。\n"
    "- 手臂水平前伸时：shoulder_lift≈0, elbow_flex≈0。\n"
    "- 举手打招呼（类似照片中挥手的动作）：shoulder_lift≈70~90, elbow_flex≈30~60。\n"
    "- 手臂下垂自然放松：shoulder_lift≈-60~-80, elbow_flex≈10~30。\n"
    "- 叉腰：elbow_flex≈90, wrist_flex≈-20。\n"
    "- 指向远处（食指指向前方）：shoulder_lift≈10~30, elbow_flex≈10~20。\n"
    "- 关节角度是连续的，请结合照片动作给出最接近的一组整数角度。"
)


def _check_key(key: str):
    """校验 API Key：不能为空、不能含中文/占位符（如'你的key'）。"""
    if not key:
        sys.exit("[error] 未找到 API Key。请设置 ZHIPU_API_KEY / SILICONFLOW_API_KEY / OPENAI_API_KEY 环境变量")
    if "你的" in key or "xxxx" in key or any(ord(c) > 127 for c in key):
        sys.exit("[error] API Key 无效：包含中文或占位符（如'你的key'）。请填入从平台复制的真实 key。")


def make_client():
    """根据环境变量选择 provider，返回 (OpenAI client, model)。"""
    from openai import OpenAI
    key = os.environ.get("ZHIPU_API_KEY") or os.environ.get("SILICONFLOW_API_KEY") or os.environ.get("OPENAI_API_KEY")
    _check_key(key)
    if os.environ.get("ZHIPU_API_KEY"):
        base = os.environ.get("ZHIPU_BASE_URL", "https://open.bigmodel.cn/api/paas/v4/")
        model = os.environ.get("ZHIPU_MODEL", "glm-4.6v-flash")
        print(f"[vlm] provider=zhipu model={model}")
    elif os.environ.get("SILICONFLOW_API_KEY"):
        base = os.environ.get("SILICONFLOW_BASE_URL", "https://api.siliconflow.cn/v1")
        model = os.environ.get("SILICONFLOW_MODEL", "Qwen/Qwen3.5-4B")
        print(f"[vlm] provider=siliconflow model={model}")
    else:
        base = os.environ.get("OPENAI_BASE_URL")
        model = os.environ.get("OPENAI_MODEL", "gpt-4o-mini")
        print(f"[vlm] provider=openai model={model}")
    client = OpenAI(api_key=key, base_url=base or None)
    return client, model


def step_capture(camera_id: int) -> Path:
    """任务一：拍照保存（不 imshow）。提示用户摆好姿势，按回车后 3 秒倒计时拍摄。"""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        sys.exit(f"[error] 无法打开摄像头 #{camera_id}（确认相机已连接）")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # 等 10 帧让自动曝光稳定
    for _ in range(10):
        cap.read()
    try:
        input(">>> 请摆好动作，按回车后 3 秒拍照（Ctrl+C 可取消）")
    except KeyboardInterrupt:
        cap.release()
        sys.exit("\n已取消拍照。")
    for i in (3, 2, 1):
        print(f"    {i}...")
        time.sleep(1)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        sys.exit("[error] 读取摄像头失败")
    path = SHOTS_DIR / f"arm_{time.strftime('%H%M%S')}.jpg"
    cv2.imwrite(str(path), frame)
    print(f"[capture] 已拍照: {path}")
    return path


def step_vlm(image_path: Path, level: int = 1, client=None, model=None) -> dict:
    """任务二：VLM 解析照片中的手臂动作 → 动作描述 + 关节参数 JSON。"""
    if not image_path.exists():
        sys.exit(f"[error] 照片不存在: {image_path}")
    if client is None or model is None:
        client, model = make_client()
    data_url = "data:image/jpeg;base64," + base64.b64encode(image_path.read_bytes()).decode()
    prompt = PROMPT_L2 if level >= 2 else PROMPT_L1
    completion = client.chat.completions.create(
        model=model,
        messages=[{
            "role": "user",
            "content": [
                {"type": "image_url", "image_url": {"url": data_url}},
                {"type": "text", "text": prompt},
            ],
        }],
        temperature=0.2,
        max_tokens=1200,
    )
    content = completion.choices[0].message.content or ""
    print("[vlm] 模型输出:\n" + content)
    # 提取 JSON（容忍 ```json 包裹）
    import re
    m = re.search(r"\{.*\}", content, re.S)
    if not m:
        sys.exit("[error] VLM 输出未包含 JSON，请重试或更换模型")
    return json.loads(m.group(0))


def step_drive(joints: dict, dry_run: bool, port: str) -> dict:
    """任务三：把关节参数发给机械臂执行。"""
    from arm_tools import ArmController
    arm = ArmController(port=port, dry_run=dry_run)
    try:
        result = arm.move_to_joints(joints)
        time.sleep(2.0)  # 留足运动时间
        print(f"[drive] 执行结果: {json.dumps(result, ensure_ascii=False)}")
        return result
    finally:
        arm.close()


def main():
    p = argparse.ArgumentParser(description="Week7 4.1 机械臂模仿你的动作 pipeline")
    p.add_argument("--step", choices=["all", "capture", "vlm", "drive"], default="all",
                   help="执行步骤（默认 all=拍照→解析→驱动）")
    p.add_argument("--camera-id", type=int, default=1, help="相机编号（默认 1=腕部相机）")
    p.add_argument("--image", type=Path, default=None, help="vlm 步骤用：指定照片路径")
    p.add_argument("--joints", type=str, default=None, help="drive 步骤用：关节 JSON 字符串")
    p.add_argument("--level", type=int, default=1, choices=[1, 2],
                   help="prompt 级别：1=基础（任务三），2=增强含示例（任务四）")
    p.add_argument("--dry-run", action="store_true", help="仿真模式：不连机械臂")
    p.add_argument("--port", default="COM3")
    args = p.parse_args()

    client, model = (None, None)
    image_path = args.image
    joints = None

    if args.step in ("all", "capture"):
        image_path = step_capture(args.camera_id)
    if args.step in ("all", "vlm"):
        if image_path is None:
            p.error("--step vlm 需要 --image 指定照片")
        result = step_vlm(image_path, level=args.level, client=client, model=model)
        joints = result.get("joints")
        if joints is None:
            sys.exit(f"[error] VLM 输出缺少 joints 字段: {result}")
        print(f"[vlm] 动作描述: {result.get('description', '')}")
        print(f"[vlm] 关节参数: {json.dumps(joints, ensure_ascii=False)}")
    if args.step in ("all", "drive"):
        if joints is None and args.joints:
            joints = json.loads(args.joints)
        if joints is None:
            p.error("--step drive 需要 --joints 或先执行 vlm 步骤")
        step_drive(joints, dry_run=args.dry_run, port=args.port)

    print("\n完成：拍照 → 调用大模型 → 控制机械臂 全流程已执行。" if args.step == "all"
          else f"完成步骤 {args.step}。")


if __name__ == "__main__":
    main()
