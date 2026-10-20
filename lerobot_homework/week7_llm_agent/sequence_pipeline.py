"""
Week7 任务五：文本命令 → 机械臂连贯动作

用文本大模型（DeepSeek deepseek-chat）把自然语言命令解析为动作序列，
机械臂在不经过人为干预的情况下连续执行多个姿态（验收：≥2 个不同姿态的连贯动作）。

运行：
  $env:OPENAI_API_KEY="sk-..."
  $env:OPENAI_BASE_URL="https://api.deepseek.com"
  $env:OPENAI_MODEL="deepseek-chat"
  python sequence_pipeline.py --command "先挥手再点头"
  python sequence_pipeline.py                          # 交互输入命令
  python sequence_pipeline.py --command "挥手" --dry-run   # 仿真验证不连机械臂
"""
import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
ACTIONS_LIB = json.loads((BASE_DIR / "actions_library.json").read_text(encoding="utf-8"))
FRAME_PAUSE = ACTIONS_LIB.get("帧间停顿秒", 0.9)
# 动作库：动作名 -> 关节帧序列（依序执行即形成一个连贯动作）。数据在 actions_library.json，可直接编辑。
ACTIONS = {name: entry["帧"] for name, entry in ACTIONS_LIB["动作"].items()}


def _check_key(key: str):
    if not key:
        sys.exit("[error] 未找到 API Key。请设置 ZHIPU_API_KEY / SILICONFLOW_API_KEY / OPENAI_API_KEY 环境变量")
    if "你的" in key or "xxxx" in key or any(ord(c) > 127 for c in key):
        sys.exit("[error] API Key 无效：包含中文或占位符（如'你的key'）。请填入从平台复制的真实 key。")


def make_client():
    """文本模型 client：优先 ZHIPU / SILICONFLOW / OPENAI（DeepSeek 走 OPENAI 兼容）。"""
    from openai import OpenAI
    key = os.environ.get("ZHIPU_API_KEY") or os.environ.get("SILICONFLOW_API_KEY") or os.environ.get("OPENAI_API_KEY")
    _check_key(key)
    if os.environ.get("ZHIPU_API_KEY"):
        base, model = os.environ.get("ZHIPU_BASE_URL", "https://open.bigmodel.cn/api/paas/v4/"), os.environ.get("ZHIPU_MODEL", "glm-4-flash")
    elif os.environ.get("SILICONFLOW_API_KEY"):
        base, model = os.environ.get("SILICONFLOW_BASE_URL", "https://api.siliconflow.cn/v1"), os.environ.get("SILICONFLOW_MODEL", "Qwen/Qwen3-8B")
    else:
        base, model = os.environ.get("OPENAI_BASE_URL"), os.environ.get("OPENAI_MODEL", "deepseek-chat")
    print(f"[llm] provider=text model={model}")
    return OpenAI(api_key=key, base_url=base or None), model


def parse_command(command: str, client=None, model=None):
    """文本模型把命令解析成动作序列；无 key/解析失败时回退关键词匹配。"""
    if client is not None and model is not None:
        try:
            sys_prompt = ("你是机械臂动作规划器。把用户的中文动作命令解析成动作序列，"
                          "只输出 JSON（不要解释），格式：{\"sequence\": [\"动作名\", \"动作名\", ...]}。"
                          "动作名只能从以下列表选择：" + "、".join(ACTIONS.keys()) + "。"
                          "没有对应动作时输出 {\"sequence\": []}。")
            r = client.chat.completions.create(
                model=model, temperature=0.2, max_tokens=200,
                messages=[{"role": "system", "content": sys_prompt},
                          {"role": "user", "content": command}])
            content = r.choices[0].message.content or ""
            m = re.search(r"\{.*\}", content, re.S)
            if m:
                seq = json.loads(m.group(0)).get("sequence", [])
                if seq:
                    print(f"[llm] 动作序列: {seq}")
                    return [a for a in seq if a in ACTIONS]
        except Exception as e:
            print(f"[llm] 解析失败（回退关键词匹配）: {type(e).__name__}: {str(e)[:120]}")
    # 关键词回退
    seq = [name for name in ACTIONS if name in command]
    print(f"[fallback] 关键词匹配: {seq}")
    return seq


def run_sequence(seq, dry_run, port):
    from arm_tools import ArmController
    arm = ArmController(port=port, dry_run=dry_run)
    try:
        for name in seq:
            frames = ACTIONS[name]
            print(f"\n[action] 执行『{name}』（{len(frames)} 帧）")
            for i, joints in enumerate(frames, 1):
                result = arm.move_to_joints(joints)
                print(f"  帧{i}/{len(frames)} {json.dumps(joints, ensure_ascii=False)} -> {result.get('ok')}")
                time.sleep(FRAME_PAUSE)
    finally:
        arm.close()


def main():
    p = argparse.ArgumentParser(description="Week7 任务五：文本命令→机械臂连贯动作")
    p.add_argument("--command", type=str, default=None, help="动作命令，如'先挥手再点头'；缺省则交互输入")
    p.add_argument("--dry-run", action="store_true", help="仿真模式：不连机械臂")
    p.add_argument("--port", default="COM3")
    args = p.parse_args()

    command = args.command
    if not command:
        try:
            command = input(">>> 输入动作命令（如：先挥手再点头）：").strip()
        except EOFError:
            sys.exit("[error] 未提供命令")

    client, model = (None, None)
    if not args.dry_run:
        client, model = make_client()
    seq = parse_command(command, client, model)
    if not seq:
        sys.exit("[error] 未能从命令中识别出任何动作（可用：" + "、".join(ACTIONS.keys()) + "）")

    run_sequence(seq, dry_run=args.dry_run, port=args.port)
    print("\n完成：连贯动作已执行（无人工干预）。")


if __name__ == "__main__":
    main()
