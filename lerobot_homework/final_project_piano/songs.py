# -*- coding: utf-8 -*-
"""内置简谱与音符解析。"""

from __future__ import annotations

import re

MELODIES = {
    "scale": ["1", "2", "3", "4", "5", "6", "7"],
    "twinkle": [
        "1", "1", "5", "5", "6", "6", "5",
        "4", "4", "3", "3", "2", "2", "1",
    ],
    "ode": ["3", "3", "4", "5", "5", "4", "3", "2", "1", "1", "2", "3", "3", "2", "2"],
    "twinkle_full": (
        ["1", "1", "5", "5", "6", "6", "5",
         "4", "4", "3", "3", "2", "2", "1",
         "5", "5", "4", "4", "3", "3", "2",
         "5", "5", "4", "4", "3", "3", "2",
         "1", "1", "5", "5", "6", "6", "5",
         "4", "4", "3", "3", "2", "2", "1"]
    ),
    "habagou": (
        ["1", "1", "1", "2", "3",
         "3", "3", "3", "4", "5",
         "6", "6", "5", "4", "3",
         "5", "5", "2", "3", "1"] * 2
    ),
}

MELODY_BEATS = {
    "twinkle_full": [
        1, 1, 1, 1, 1, 1, 2,
        1, 1, 1, 1, 1, 1, 2,
        1, 1, 1, 1, 1, 1, 2,
        1, 1, 1, 1, 1, 1, 2,
        1, 1, 1, 1, 1, 1, 2,
        1, 1, 1, 1, 1, 1, 2,
    ],
    "habagou": [
        0.5, 0.5, 0.5, 0.5, 2,
        0.5, 0.5, 0.5, 0.5, 2,
        0.5, 0.5, 0.5, 0.5, 2,
        0.5, 0.5, 0.5, 0.5, 2,
    ] * 2,
}


def parse_notes(text: str, available: list[str]) -> list[str]:
    normalized = {
        "do": "1", "re": "2", "mi": "3", "fa": "4",
        "sol": "5", "la": "6", "si": "7",
        "高do": "1高", "highdo": "1高",
        "c4": "1", "d4": "2", "e4": "3", "f4": "4",
        "g4": "5", "a4": "6", "b4": "7", "c5": "1高",
    }
    result = []
    for token in re.split(r"[\s,，、]+", text.strip()):
        if not token:
            continue
        note = normalized.get(token.lower(), token)
        if note not in available:
            raise ValueError(
                f"未知音键：{token}；当前可用音键：{', '.join(available)}"
            )
        result.append(note)
    return result
