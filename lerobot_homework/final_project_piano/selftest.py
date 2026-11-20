# -*- coding: utf-8 -*-
"""不连接机械臂的配置和乐谱自检。"""

from __future__ import annotations

from pathlib import Path

import common
from common import arm_lib
from songs import MELODIES, MELODY_BEATS


def main() -> int:
    ranges = arm_lib.load_ranges(common.DEFAULT_RANGES)
    poses = common.load_json(common.CONFIG_DIR / "piano_joint_poses.json")
    keys = poses["keys"]
    hover = poses["hover"]
    assert all(str(index) in keys for index in range(1, 8))
    for name, steps in [("hover", hover), *[(f"key{key}", value) for key, value in keys.items()]]:
        assert set(steps) == set("123456"), (name, steps)
        for sid_text, position in steps.items():
            low, high = ranges[int(sid_text)]
            assert low <= int(position) <= high, (name, sid_text, position)
    for song in ("twinkle_full", "habagou"):
        assert len(MELODIES[song]) == len(MELODY_BEATS[song])
        assert set(MELODIES[song]).issubset(set(keys))
    print("配置、1~7 音键和内置乐谱自检通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
