#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Week7 4.2：LLM Agent —— 把机械臂能力作为 Tools，用外部 LLM API 完成 function calling。

Agent Loop（ReAct）：
  用户自然语言任务 → LLM 决定调用哪些工具（get_robot_state / detect_object /
  move_to_joints / move_to_pose / start_tracking / stop_tracking）
  → Agent 程序执行真实函数 → 把结果返回 LLM → LLM 继续推理直到给出最终回答。

用法：
  # dry-run 仿真（不连机械臂，验证 function calling 流程）
  python llm_agent.py --dry-run --task "检测黑色小球，然后追踪它"
  # 真机
  python llm_agent.py --task "看看机械臂当前状态，然后把末端移到 (0.2, 0, 0.3)"
  # 交互模式
  python llm_agent.py --interactive

环境变量：同 imitate_pipeline.py（ZHIPU_API_KEY / SILICONFLOW_API_KEY / OPENAI_API_KEY）。
"""
import argparse
import json
import os
import sys

from arm_tools import ArmController, tools_schema

SYSTEM_PROMPT = (
    "你是 SO-ARM101 六自由度机械臂的控制 Agent。你通过调用工具来感知环境和执行动作。\n"
    "规则：\n"
    "1. 移动机械臂前先调用 get_robot_state 了解当前状态；\n"
    "2. 追踪任务：先 detect_object 找到物体（返回 object_id），再 start_tracking(object_id)；\n"
    "3. 关节角度单位是度，必须落在各关节范围内（工具说明里已给出）；\n"
    "4. 危险/含糊的指令不要执行，直接说明原因；\n"
    "5. 用简洁中文总结你做了什么。"
)


def make_client():
    from openai import OpenAI
    key = os.environ.get("ZHIPU_API_KEY") or os.environ.get("SILICONFLOW_API_KEY") or os.environ.get("OPENAI_API_KEY")
    if not key:
        sys.exit("[error] 未找到 API Key：请设置 ZHIPU_API_KEY / SILICONFLOW_API_KEY / OPENAI_API_KEY")
    if os.environ.get("ZHIPU_API_KEY"):
        base = os.environ.get("ZHIPU_BASE_URL", "https://open.bigmodel.cn/api/paas/v4/")
        model = os.environ.get("ZHIPU_MODEL", "glm-4.6v-flash")
    elif os.environ.get("SILICONFLOW_API_KEY"):
        base = os.environ.get("SILICONFLOW_BASE_URL", "https://api.siliconflow.cn/v1")
        model = os.environ.get("SILICONFLOW_MODEL", "Qwen/Qwen3.5-4B")
    else:
        base = os.environ.get("OPENAI_BASE_URL")
        model = os.environ.get("OPENAI_MODEL", "gpt-4o-mini")
    print(f"[agent] provider={base or 'openai'} model={model}")
    return OpenAI(api_key=key, base_url=base or None), model


def run_agent(client, model, task: str, arm: ArmController, max_rounds: int = 10):
    """ReAct loop：LLM 决策 → 执行工具 → 反馈 → 直到无工具调用。"""
    tools = tools_schema()
    messages = [{"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": task}]
    print(f"\n[agent] 任务: {task}")
    for rnd in range(1, max_rounds + 1):
        resp = client.chat.completions.create(
            model=model, messages=messages, tools=tools, tool_choice="auto", temperature=0)
        msg = resp.choices[0].message
        if not msg.tool_calls:
            print(f"[agent] 最终回答: {msg.content}")
            return msg.content
        messages.append(msg)  # assistant 的工具调用请求
        for tc in msg.tool_calls:
            fn = tc.function.name
            args = json.loads(tc.function.arguments or "{}")
            print(f"[round {rnd}] 调用工具: {fn}({json.dumps(args, ensure_ascii=False)})")
            try:
                result = getattr(arm, fn)(**args)
            except TypeError as e:
                result = {"ok": False, "error": f"参数错误: {e}"}
            print(f"[round {rnd}] 工具返回: {json.dumps(result, ensure_ascii=False)}")
            messages.append({
                "role": "tool",
                "tool_call_id": tc.id,
                "content": json.dumps(result, ensure_ascii=False),
            })
    print("[agent] 达到最大轮数，停止。")
    return None


def main():
    p = argparse.ArgumentParser(description="Week7 4.2 LLM Agent function calling")
    p.add_argument("--task", default=None, help="自然语言任务；不填则交互模式")
    p.add_argument("--interactive", action="store_true", help="交互模式（每轮输入任务）")
    p.add_argument("--dry-run", action="store_true", help="仿真模式：不连机械臂")
    p.add_argument("--port", default="COM3")
    p.add_argument("--max-rounds", type=int, default=10)
    args = p.parse_args()

    client, model = make_client()
    arm = ArmController(port=args.port, dry_run=args.dry_run)
    try:
        if args.task:
            run_agent(client, model, args.task, arm, args.max_rounds)
        elif args.interactive:
            print("交互模式：输入任务（如 '检测黑色小球并追踪它'），输入 exit 退出")
            while True:
                try:
                    task = input(">>> ").strip()
                except EOFError:
                    break
                if not task:
                    continue
                if task.lower() in ("exit", "quit", "q"):
                    break
                run_agent(client, model, task, arm, args.max_rounds)
        else:
            p.error("请提供 --task 或 --interactive")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
