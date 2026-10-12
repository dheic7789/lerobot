# -*- coding: utf-8 -*-
"""2.1 语音控制：语音指令 -> 文本 -> 关键词 -> 机械臂动作。

语音识别默认用 speech_recognition 的 Google 后端（zh-CN，需要联网）。
如果不想/不能开麦克风，可以加 --mode text，直接在终端输入文字代替说话。

动作支持：
  1. 预设姿态：说“姿态1”“姿势3”“回到姿态5”等，切换到 joint_poses.json 里的姿态。
  2. 预设动作：说“挥手”，执行一组姿态序列；也可以说“执行动作 1 2 3”。
  3. 末端移动：说“向前移动5厘米”“向上”“左移2厘米”等，由逆运动学解出关节角。
"""

import argparse
import json
import math
import re
import time
from pathlib import Path

import ik_control as ik


POSES_FILE = Path(__file__).with_name("joint_poses.json")
DEFAULT_DURATION = 1.5
DEFAULT_SPEED = 100
DEFAULT_VOICE_STEP = 0.03

# 挥手等“预设动作”的默认姿态序列，键名对应 joint_poses.json 的快捷键。
ACTION_ALIASES = {
    "挥手": "6 7 6 7",
}

CHINESE_DIGITS = {
    "一": "1", "衣": "1", "医": "1",
    "二": "2", "两": "2", "儿": "2", "而": "2",
    "三": "3", "散": "3",
    "四": "4", "似": "4", "寺": "4", "市": "4", "是": "4",
    "五": "5", "武": "5", "伍": "5", "午": "5", "舞": "5",
    "六": "6", "溜": "6", "陆": "6", "路": "6",
    "七": "7", "妻": "7", "棋": "7", "漆": "7",
    "八": "8", "吧": "8", "巴": "8",
    "九": "9", "久": "9", "酒": "9",
}
DIGIT_PATTERN = "[" + "".join(re.escape(key) for key in CHINESE_DIGITS) + "1-9]"

# 常见繁体字兜底（未安装 zhconv 时用）
TRAD_TO_SIMP = {
    "揮": "挥", "態": "态", "勢": "势", "歸": "归", "開": "开", "夾": "夹",
    "閉": "闭", "緊": "紧", "鬆": "松", "動": "动", "進": "进", "後": "后",
    "轉": "转", "車": "车", "執": "执", "個": "个", "號": "号", "請": "请",
    "釐": "厘", "張": "张", "關": "关", "來": "来", "這": "这", "裡": "里",
    "別": "别", "沒": "没", "點": "点", "讓": "让", "還": "还", "會": "会",
    "發": "发", "從": "从", "時": "时", "說": "说", "過": "过", "對": "对",
}

# 夹爪闭合对应舵机范围的小端还是大端；如果方向反了改成 False。
GRIPPER_CLOSE_IS_MIN = True


def load_poses(path=POSES_FILE):
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    poses = {}
    for key, pose in data.items():
        if not isinstance(pose, dict):
            continue
        positions = pose.get("positions")
        if not isinstance(positions, dict):
            continue
        poses[key] = {
            "name": pose.get("name", key),
            "positions": {int(sid): int(step) for sid, step in positions.items()},
        }
    return poses


def workspace_bounds(limits):
    import random
    random.seed(0)
    zmin, zmax, rmax = 1e9, -1e9, 0.0
    for _ in range(3000):
        q = {}
        for sid in range(1, 6):
            lo, hi = limits[sid]
            q[ik.JOINT_BY_ID[sid]] = random.uniform(lo, hi)
        p = ik.fk_position(q)
        zmin = min(zmin, p[2])
        zmax = max(zmax, p[2])
        rmax = max(rmax, math.hypot(p[0], p[1]))
    return zmin, zmax, rmax


def _extract_distance(text, default_step):
    match = re.search(r"(-?\d+(?:\.\d+)?)\s*(厘米|公分|cm|米|m)?", text)
    if match is None:
        return default_step
    value = abs(float(match.group(1)))
    unit = (match.group(2) or "").lower()
    if unit in ("厘米", "公分", "cm"):
        value /= 100.0
    return max(0.0, min(0.5, value))


def to_simplified(text):
    """Whisper 常输出繁体字，转成简体再匹配关键词。"""
    try:
        from zhconv import convert
        return convert(text, "zh-cn")
    except ImportError:
        return "".join(TRAD_TO_SIMP.get(ch, ch) for ch in text)


def _detect_motion(text, step):
    """带否定和“反方向”处理的方向识别。"""
    if any(word in text for word in ("不要", "别", "停止", "停下", "暂停", "别动")):
        return ("stop",)

    if any(word in text for word in ("反方向", "反向", "相反方向")):
        if "左" in text:
            direction = "right"
        elif "右" in text:
            direction = "left"
        elif "前" in text:
            direction = "back"
        elif "后" in text:
            direction = "forward"
        elif "上" in text:
            direction = "down"
        elif "下" in text:
            direction = "up"
        else:
            direction = None
        if direction:
            return ("move_dir", direction, _extract_distance(text, step))

    direction_map = [
        ("向前", "forward"), ("前进", "forward"), ("往前", "forward"), ("前", "forward"),
        ("向后", "back"), ("后退", "back"), ("倒车", "back"), ("往后", "back"), ("后", "back"),
        ("向左", "left"), ("左转", "left"), ("往左", "left"), ("左", "left"),
        ("向右", "right"), ("右转", "right"), ("往右", "right"), ("右", "right"),
        ("向上", "up"), ("往上", "up"), ("上", "up"),
        ("向下", "down"), ("往下", "down"), ("下", "down"),
    ]
    for word, direction in direction_map:
        if word in text:
            return ("move_dir", direction, _extract_distance(text, step))
    return None


def _extract_pose_key(text, poses):
    pose_words = (
        "姿态", "姿势", "位姿", "pose", "pos",
        "自拍", "子台", "字台", "紫台", "姿太", "四台",
        "字太", "子太", "资态", "志台", "自太", "滋太",
    )
    for word in pose_words:
        match = re.search(word + r"[，,\s]*(" + DIGIT_PATTERN + ")", text)
        if match:
            key = CHINESE_DIGITS.get(match.group(1), match.group(1))
            if key in poses:
                return key

    match = re.search(r"第[，,\s]*(" + DIGIT_PATTERN + r")[，,\s]*(?:个|号)?[，,\s]*(?:姿态|姿势)?", text)
    if match:
        key = CHINESE_DIGITS.get(match.group(1), match.group(1))
        if key in poses:
            return key

    match = re.search(r"(" + DIGIT_PATTERN + r")[，,\s]*号[，,\s]*(?:姿态|姿势)", text)
    if match:
        key = CHINESE_DIGITS.get(match.group(1), match.group(1))
        if key in poses:
            return key

    match = re.search(r"(?:执行|切换|转到|去|到|播放|进入)[，,\s]*(" + DIGIT_PATTERN + ")", text)
    if match:
        key = CHINESE_DIGITS.get(match.group(1), match.group(1))
        if key in poses:
            return key
    return None


def _detect_gripper(text):
    if any(word in text for word in ("合上", "闭合", "关闭", "夹紧", "抓住", "握紧", "夹住")):
        return "close"
    if any(word in text for word in ("打开", "张开", "松开", "放开", "伸展", "松开夹爪")):
        return "open"
    return None


def parse_voice_command(text, poses, step):
    t = text.strip().lower()
    if not t:
        return ("unknown", "")

    if any(word in t for word in ("回中", "归中", "归零", "复位", "回到中间", "初始", "home", "立正", "站好", "站直")):
        return ("home",)

    pose_key = _extract_pose_key(t, poses)
    if pose_key is not None:
        return ("pose", pose_key)

    if "挥手" in t:
        return ("action", "挥手")
    match = re.search(r"(?:执行|播放|运行|跑)\s*动作\s*([1-9][1-9\s,，]*)", t)
    if match:
        return ("action", re.sub(r"[\s,，]", "", match.group(1)))
    match = re.search(r"动作\s*([1-9][1-9]*)", t)
    if match:
        return ("action", match.group(1))

    gripper = _detect_gripper(t)
    if gripper is not None:
        return ("gripper", gripper)

    if any(word in t for word in ("弯腰", "玩腰", "弯下腰", "鞠躬", "低头")):
        return ("move_vec", 0.03, 0.0, -0.05)
    if any(word in t for word in ("抬头", "起来", "起身")):
        return ("move_vec", -0.02, 0.0, 0.03)

    if any(word in t for word in ("停下", "停止", "暂停", "别动")):
        return ("stop",)

    motion = _detect_motion(t, step)
    if motion is not None:
        return motion

    command = ik.parse_command(t, step)
    if command[0] in ("move_dir", "move_vec", "goto"):
        return command
    return ("unknown", t)


def _record_audio(mic_index=None):
    try:
        import speech_recognition as sr
    except ImportError:
        raise RuntimeError(
            "未安装 speech_recognition。请先执行 pip install SpeechRecognition pyaudio，"
            "或使用 --mode text 用打字代替语音。"
        )

    recognizer = sr.Recognizer()
    recognizer.dynamic_energy_threshold = True
    recognizer.dynamic_energy_adjustment_damping = 0.15
    recognizer.dynamic_energy_ratio = 1.2
    recognizer.pause_threshold = 0.8
    recognizer.energy_threshold = 150
    try:
        mic = sr.Microphone(device_index=mic_index) if mic_index is not None else sr.Microphone()
        with mic as source:
            print("请说话...", flush=True)
            recognizer.adjust_for_ambient_noise(source, duration=0.6)
            audio = recognizer.listen(source, timeout=10, phrase_time_limit=10)
    except Exception as error:
        try:
            names = sr.Microphone.list_microphone_names()
        except Exception:
            names = []
        hint = ("可用麦克风：" + " | ".join(f"{i}:{n}" for i, n in enumerate(names))) if names else ""
        raise RuntimeError(f"无法使用麦克风：{error}。{hint}") from error
    return recognizer, audio


def _transcribe_whisper(audio, model):
    import numpy as np

    pcm = audio.get_raw_data(convert_rate=16000, convert_width=2)
    samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
    segments, _ = model.transcribe(
        samples,
        language="zh",
        beam_size=5,
        vad_filter=True,
        condition_on_previous_text=False,
    )
    return "".join(segment.text for segment in segments).strip()


def resolve_stt(choice):
    if choice != "auto":
        return choice
    try:
        import faster_whisper  # noqa: F401
        return "whisper"
    except ImportError:
        pass
    try:
        import speech_recognition  # noqa: F401
        return "google"
    except ImportError:
        return "text"


def listen_once(backend, whisper_model, mic_index=None):
    recognizer, audio = _record_audio(mic_index)
    if backend == "whisper":
        return _transcribe_whisper(audio, whisper_model)
    if backend == "google":
        return recognizer.recognize_google(audio, language="zh-CN")
    if backend == "sphinx":
        return recognizer.recognize_sphinx(audio, language="zh-CN")
    raise ValueError(f"未知语音后端：{backend}")


def _to_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _motion_to_command(motion):
    if not isinstance(motion, dict):
        return None
    kind = motion.get("kind") or motion.get("action")
    if kind == "home":
        return ("home",)
    if kind == "pose":
        return ("pose", str(motion.get("key", "")).strip())
    if kind == "action":
        return ("action", str(motion.get("name", "")).strip())
    if kind == "gripper":
        state = str(motion.get("state", "")).lower()
        return ("gripper", state if state in ("open", "close") else "close")
    if kind in ("move", "move_relative"):
        dx = _to_float(motion.get("dx"), 0.0)
        dy = _to_float(motion.get("dy"), 0.0)
        dz = _to_float(motion.get("dz"), 0.0)
        return ("move_vec", max(-0.5, min(0.5, dx)), max(-0.5, min(0.5, dy)), max(-0.5, min(0.5, dz)))
    if kind == "move_dir":
        direction = str(motion.get("direction", "")).lower()
        distance = max(0.0, min(0.5, _to_float(motion.get("distance_m"), 0.03)))
        if direction in ("forward", "back", "left", "right", "up", "down"):
            return ("move_dir", direction, distance)
    if kind == "goto":
        x = _to_float(motion.get("x"), 0.0)
        y = _to_float(motion.get("y"), 0.0)
        z = _to_float(motion.get("z"), 0.0)
        return ("goto", x, y, z)
    if kind == "pause":
        return ("pause", max(0.0, min(5.0, _to_float(motion.get("seconds"), 0.5))))
    if kind == "stop":
        return ("stop",)
    return None


def llm_to_motions(obj):
    if not isinstance(obj, dict):
        return []
    motions = obj.get("motions")
    if isinstance(motions, list):
        commands = [_motion_to_command(m) for m in motions]
        return [c for c in commands if c is not None]
    target_commands = _target_to_commands(obj)
    if target_commands:
        return target_commands
    command = _motion_to_command(obj)
    return [command] if command is not None else []


def _target_to_commands(obj):
    commands = []
    if obj.get("home"):
        commands.append(("home",))
    if obj.get("stop"):
        commands.append(("stop",))
    gripper = str(obj.get("gripper", "keep")).lower()
    if gripper in ("open", "close"):
        commands.append(("gripper", gripper))
    x = obj.get("x", obj.get("target_x"))
    y = obj.get("y", obj.get("target_y"))
    z = obj.get("z", obj.get("target_z"))
    if x is not None and y is not None and z is not None:
        commands.append(("goto", _to_float(x, 0.0), _to_float(y, 0.0), _to_float(z, 0.0)))
    return commands


def llm_parse(text, context=""):
    import os
    try:
        import openai
    except ImportError:
        return None
    deepseek_key = os.environ.get("DEEPSEEK_API_KEY")
    api_key = deepseek_key or os.environ.get("OPENAI_API_KEY")
    if not api_key:
        return None
    if deepseek_key:
        base_url = os.environ.get("DEEPSEEK_BASE_URL", "https://api.deepseek.com")
        model = os.environ.get("DEEPSEEK_MODEL", "deepseek-chat")
    else:
        base_url = os.environ.get("OPENAI_BASE_URL")
        model = os.environ.get("OPENAI_MODEL", "gpt-4o-mini")
    client = openai.OpenAI(api_key=api_key, base_url=base_url or None)
    prompt = (
        "你是机械臂控制器。已知机械臂末端当前位置，用户用自然语言给一个指令，"
        "你只需要给出“目标状态”，只输出 JSON，不要解释。"
        "JSON 字段：target_x、target_y、target_z（末端目标绝对坐标，单位米；x=前、y=左、z=上）；"
        "gripper（keep/open/close，默认 keep）；home（true/false）；stop（true/false）。"
        "把用户的相对说法换算成绝对目标坐标。"
        "示例："
        "“弯腰”“玩腰” -> {\"target_x\":当前x+0.03,\"target_y\":当前y,\"target_z\":当前z-0.05}；"
        "“向上5厘米” -> {\"target_x\":当前x,\"target_y\":当前y,\"target_z\":当前z+0.05}；"
        "“向左3厘米” -> {\"target_x\":当前x,\"target_y\":当前y+0.03,\"target_z\":当前z}；"
        "“立正”“归中”“回中”“归零” -> {\"home\":true}；"
        "“停止”“停下”“别动” -> {\"stop\":true}；"
        "“合上夹爪”“抓住”“夹紧” -> {\"gripper\":\"close\"}；"
        "“打开夹爪”“松开”“张开” -> {\"gripper\":\"open\"}。"
        "注意：输入可能是语音识别同音错别字，例如“玩腰”是“弯腰”，请按读音和上下文推断真实意图。"
        "如果只改变夹爪或回中/停止，就不要输出 target_x/y/z。"
    )
    if context:
        prompt = "当前机械臂状态：" + context + "。\n" + prompt
    resp = client.chat.completions.create(
        model=model,
        messages=[{"role": "user", "content": prompt + "\n文本：" + text}],
        temperature=0,
        max_tokens=800,
        response_format={"type": "json_object"},
    )
    content = resp.choices[0].message.content.strip()
    content = re.sub(r"^```(?:json)?|```$", "", content).strip()
    return json.loads(content)


def llm_ready():
    import os
    try:
        import openai  # noqa: F401
    except ImportError:
        return False
    return bool(os.environ.get("DEEPSEEK_API_KEY") or os.environ.get("OPENAI_API_KEY"))


def llm_missing_reason():
    import os
    try:
        import openai  # noqa: F401
    except ImportError:
        return "未安装 openai 包（请执行 pip install openai）"
    if not (os.environ.get("DEEPSEEK_API_KEY") or os.environ.get("OPENAI_API_KEY")):
        return "未设置 DEEPSEEK_API_KEY 环境变量"
    return None


def main():
    parser = argparse.ArgumentParser(description="2.1 语音控制机械臂")
    parser.add_argument("--port", default=ik.PORT, help=f"串口号（默认 {ik.PORT}）")
    parser.add_argument("--dry-run", action="store_true",
                        help="不连接舵机，只打印识别结果和将执行的动作")
    parser.add_argument("--mode", choices=["voice", "text"], default="voice",
                        help="voice=麦克风语音，text=终端打字（默认 voice）")
    parser.add_argument("--stt", choices=["auto", "google", "sphinx", "whisper"], default="auto",
                        help="语音识别后端：auto=自动选择，whisper=本地 Whisper（推荐），google=联网 Google")
    parser.add_argument("--whisper-model", default="small",
                        help="Whisper 模型名，可选 tiny/base/small/medium（默认 small）")
    parser.add_argument("--mic-index", type=int, default=None,
                        help="麦克风编号（0 开始）；不指定则用系统默认麦克风")
    parser.add_argument("--llm", action="store_true",
                        help="关键词匹配失败时，调用 OpenAI 兼容 LLM 做更智能的指令解析")
    parser.add_argument("--no-llm", action="store_true",
                        help="强制关闭 LLM，只用本地关键词匹配")
    parser.add_argument("--step", type=float, default=DEFAULT_VOICE_STEP,
                        help=f"语音只说方向不说距离时的默认步长，单位米（默认 {DEFAULT_VOICE_STEP:g}）")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help=f"每段动作时长，单位秒（默认 {DEFAULT_DURATION:g}）")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help=f"速度寄存器值（默认 {DEFAULT_SPEED}）")
    args = parser.parse_args()

    if args.no_llm:
        args.llm = False
    elif args.llm:
        reason = llm_missing_reason()
        if reason is None:
            print("LLM 已启用：DeepSeek（deepseek-chat）")
        else:
            print(f"LLM 未就绪：{reason}。本次回退到关键词模式。")
            args.llm = False
    elif llm_ready():
        args.llm = True
        print("检测到 DeepSeek 配置，已自动启用 LLM。")

    try:
        ranges = ik.load_ranges()
    except ValueError as error:
        parser.error(str(error))
    poses = load_poses()

    mids = ik.ranges_midpoints(ranges)
    limits = ik.ranges_limits_rad(ranges, mids)
    bounds = workspace_bounds(limits)

    backend = "text" if args.mode == "text" else resolve_stt(args.stt)
    whisper_model = None
    if args.mode == "voice":
        print(f"语音后端：{backend}")
        if backend == "whisper":
            try:
                from faster_whisper import WhisperModel
                whisper_model = WhisperModel(args.whisper_model, device="cpu", compute_type="int8")
            except Exception as error:
                parser.error(
                    f"无法加载 Whisper 模型：{error}。"
                    "可先执行 pip install faster-whisper，或改用 --stt google / --mode text。"
                )
        elif backend == "text":
            print("未安装语音识别库，自动退回文字模式。")
            args.mode = "text"

    ser = None
    try:
        if args.dry_run:
            steps = ik.home_steps(ranges)
        else:
            import serial
            ser = serial.Serial(args.port, ik.BAUDRATE, timeout=0.2)
            for sid in sorted(ranges):
                ik.set_torque(ser, sid, True)
            steps = ik.read_real_steps(ser, ranges)
            print(f"已连接 {args.port} 并使能六个舵机；退出后扭矩保持使能，请支撑机械臂。")
    except Exception as error:
        if ser is not None and getattr(ser, "is_open", False):
            ser.close()
        if isinstance(error, ModuleNotFoundError):
            parser.error("未安装 pyserial，请先执行 pip install pyserial")
        parser.error(str(error))

    try:
        current_rad = ik.steps_to_joint_rad(steps, mids)
        held_steps = {sid: steps[sid] for sid in ik.HELD_IDS}

        def move_steps(target_steps):
            if ser is not None:
                ik.sync_move(ser, target_steps, args.duration, args.speed)
            for sid in sorted(target_steps):
                print(f"  {ik.JOINT_BY_ID[sid]:<14} {target_steps[sid]} 步")

        def do_pose(key):
            nonlocal steps, current_rad, held_steps
            pose = poses[key]
            positions = pose["positions"]
            target = {sid: positions.get(sid, steps[sid]) for sid in sorted(ranges)}
            print(f"  执行预设姿态：{key}（{pose['name']}）")
            move_steps(target)
            if ser is not None:
                time.sleep(args.duration)
            steps = dict(target)
            current_rad = ik.steps_to_joint_rad(steps, mids)
            held_steps = {sid: steps[sid] for sid in ik.HELD_IDS}

        def do_action(name):
            seq = ACTION_ALIASES.get(name, name)
            keys = [ch for ch in seq if ch in poses]
            if not keys:
                print(f"  未找到动作 {name} 对应的姿态。")
                return
            print(f"  执行预设动作：{name}（姿态序列 {''.join(keys)}）")
            for key in keys:
                do_pose(key)

        def do_move(command):
            nonlocal steps, current_rad, held_steps
            if command[0] == "move_dir":
                dx, dy, dz = ik.direction_to_vector(command[1], command[2])
                x, y, z = ik.current_end_position(current_rad)
                target = (x + dx, y + dy, z + dz)
            elif command[0] == "move_vec":
                x, y, z = ik.current_end_position(current_rad)
                target = (x + command[1], y + command[2], z + command[3])
            elif command[0] == "goto":
                target = (command[1], command[2], command[3])
            else:
                return

            q, note = ik.inverse_kinematics(target, current_rad, limits)
            if note:
                print(f"  {note}")
            new_steps = dict(held_steps)
            for sid in ik.ACTIVE_IDS:
                name = ik.JOINT_BY_ID[sid]
                step = ik.rad_to_step(q[name], mids[sid])
                lo, hi = ranges[sid]
                new_steps[sid] = max(lo, min(hi, step))
            achieved = ik.fk_position(q)
            print(f"  末端参考点位置: x={achieved[0]:+.3f} m, y={achieved[1]:+.3f} m, z={achieved[2]:+.3f} m")
            move_steps(new_steps)
            steps = dict(new_steps)
            current_rad = q
            held_steps = {sid: steps[sid] for sid in ik.HELD_IDS}

        def do_home():
            nonlocal steps, current_rad, held_steps
            target = ik.home_steps(ranges)
            print("  回到中间位置（六个关节到校准中点）")
            move_steps(target)
            if ser is not None:
                time.sleep(args.duration)
            steps = dict(target)
            current_rad = ik.steps_to_joint_rad(steps, mids)
            held_steps = {sid: steps[sid] for sid in ik.HELD_IDS}

        def do_gripper(state):
            nonlocal steps, held_steps
            grp_min, grp_max = ranges[6]
            close_step = grp_min if GRIPPER_CLOSE_IS_MIN else grp_max
            open_step = grp_max if GRIPPER_CLOSE_IS_MIN else grp_min
            target = close_step if state == "close" else open_step
            label = "合上" if state == "close" else "打开"
            print(f"  {label}夹爪：ID6 -> {target} 步")
            if ser is not None:
                ik.sync_move(ser, {6: target}, args.duration, args.speed)
            steps[6] = target
            held_steps[6] = target

        def run_one(command):
            if command[0] == "pose":
                do_pose(command[1])
            elif command[0] == "action":
                do_action(command[1])
            elif command[0] == "home":
                do_home()
            elif command[0] == "gripper":
                do_gripper(command[1])
            elif command[0] == "stop":
                print("  停止：不发送新的运动指令。")
            elif command[0] == "pause":
                print(f"  等待 {command[1]:g} 秒")
                if ser is not None:
                    time.sleep(command[1])
            elif command[0] in ("move_dir", "move_vec", "goto"):
                do_move(command)
            else:
                print("  未识别出可执行动作。")

        def execute(text):
            text = to_simplified(text)
            print(f"  识别结果：{text}")
            command = parse_voice_command(text, poses, args.step)
            if command[0] == "unknown" and args.llm:
                try:
                    print("  LLM 理解中...", flush=True)
                    x, y, z = ik.current_end_position(current_rad)
                    context = (
                        f"末端参考点当前位置 x={x:.3f} 米(前)，y={y:.3f} 米(左)，z={z:.3f} 米(上)；"
                        f"末端可达范围：z 约 {bounds[0]:.2f}~{bounds[1]:.2f} 米，"
                        f"水平距离不超过 {bounds[2]:.2f} 米。"
                    )
                    obj = llm_parse(text, context)
                    if obj is not None:
                        motions = llm_to_motions(obj)
                        print(f"  LLM 返回：{obj}")
                        if motions:
                            for one in motions:
                                run_one(one)
                        else:
                            print("  未识别出可执行动作。")
                        return
                except Exception as error:
                    print(f"  LLM 解析失败：{error}")
            run_one(command)

        print("语音控制已启动。说“回中”“姿态1”“挥手”“向前移动5厘米”等；Ctrl+C 退出。")
        if args.mode == "text":
            print("当前为文字模式：输入文字代替说话，q 退出。")

        while True:
            try:
                if args.mode == "text":
                    text = input("语音文本> ").strip()
                    if text.lower() in ("q", "quit", "exit"):
                        break
                else:
                    text = listen_once(backend, whisper_model, args.mic_index)
            except (EOFError, KeyboardInterrupt):
                break
            except RuntimeError as error:
                print(f"  {error}")
                continue
            if not text:
                continue
            execute(text)
    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()
