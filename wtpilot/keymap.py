"""Parse War Thunder ``.blk`` control files and resolve action -> key bindings.

War Thunder stores keyboard bindings as DirectInput scan codes.  A value with
bit 7 set (>= 128) marks an extended key; ``keys.py`` strips that bit and sets
``KEYEVENTF_EXTENDEDKEY`` when injecting the stroke.
"""

from __future__ import annotations

import os
from typing import Iterable

# DirectInput scan code -> key name, limited to what an autopilot plausibly needs.
SCANCODE_NAMES: dict[int, str] = {
    1: "ESC", 2: "1", 3: "2", 4: "3", 5: "4", 6: "5", 7: "6", 8: "7", 9: "8",
    10: "9", 11: "0", 12: "MINUS", 13: "EQUALS", 14: "BACKSPACE", 15: "TAB",
    16: "Q", 17: "W", 18: "E", 19: "R", 20: "T", 21: "Y", 22: "U", 23: "I",
    24: "O", 25: "P", 26: "LBRACKET", 27: "RBRACKET", 28: "ENTER", 29: "LCTRL",
    30: "A", 31: "S", 32: "D", 33: "F", 34: "G", 35: "H", 36: "J", 37: "K",
    38: "L", 39: "SEMICOLON", 40: "APOSTROPHE", 41: "GRAVE", 42: "LSHIFT",
    43: "BACKSLASH", 44: "Z", 45: "X", 46: "C", 47: "V", 48: "B", 49: "N",
    50: "M", 51: "COMMA", 52: "PERIOD", 53: "SLASH", 54: "RSHIFT",
    55: "NUMPAD*", 56: "LALT", 57: "SPACE", 58: "CAPSLOCK",
    59: "F1", 60: "F2", 61: "F3", 62: "F4", 63: "F5", 64: "F6", 65: "F7",
    66: "F8", 67: "F9", 68: "F10", 87: "F11", 88: "F12",
    71: "NUMPAD7", 72: "NUMPAD8", 73: "NUMPAD9", 74: "NUMPAD-", 75: "NUMPAD4",
    76: "NUMPAD5", 77: "NUMPAD6", 78: "NUMPAD+", 79: "NUMPAD1", 80: "NUMPAD2",
    81: "NUMPAD3", 82: "NUMPAD0", 83: "NUMPAD.",
}
NAME_SCANCODES: dict[str, int] = {v: k for k, v in SCANCODE_NAMES.items()}

# Extended keys are named with an ``_EXT`` suffix so the mapping round-trips
# through the human-readable form used in config overrides.
EXT_SUFFIX = "_EXT"


def name_to_code(name: str) -> int | None:
    name = name.strip().upper()
    extended = name.endswith(EXT_SUFFIX)
    if extended:
        name = name[: -len(EXT_SUFFIX)]
    code = NAME_SCANCODES.get(name)
    if code is None:
        return None
    return code + 128 if extended else code

# WT hotkey id -> (autopilot action, polarity)
#   "max" end of an axis is the positive direction, "min" the negative one.
AXIS_ACTIONS: dict[str, tuple[str, str]] = {
    "elevator_rangeMax": ("elevator", "up"),
    "elevator_rangeMin": ("elevator", "down"),
    "ailerons_rangeMax": ("aileron", "right"),
    "ailerons_rangeMin": ("aileron", "left"),
    "rudder_rangeMax": ("rudder", "right"),
    "rudder_rangeMin": ("rudder", "left"),
    "throttle_rangeMax": ("throttle", "up"),
    "throttle_rangeMin": ("throttle", "down"),
}

# Action names exposed to the rest of the app.
ACTIONS = (
    "elevator_up", "elevator_down",
    "aileron_left", "aileron_right",
    "rudder_left", "rudder_right",
    "throttle_up", "throttle_down",
    "airbrake", "flaps_up", "flaps_down", "gear",
)

# Fallbacks when the .blk does not bind an action. War Thunder's own defaults.
FALLBACKS: dict[str, list[str]] = {
    "elevator_up": ["S"],
    "elevator_down": ["W"],
    "aileron_left": ["A"],
    "aileron_right": ["D"],
    # Not present in the reference key.blk, so these only apply if the user
    # rebinds rudder to the keyboard themselves.
    "rudder_left": ["Q"],
    "rudder_right": ["E"],
    "throttle_up": ["NUMPAD9"],
    "throttle_down": ["NUMPAD6"],
    "airbrake": ["G"],
    "flaps_up": ["R"],
    "flaps_down": ["F"],
    "gear": ["H"],
}


def parse_blk(text: str) -> dict[str, list[dict[str, list[str]]]]:
    """Return ``{block_name: [ {key_type: [raw values]}, ... ]}``.

    Only blocks that actually carry key/button bindings are kept.  The same
    block name may repeat (WT allows several bindings per action), hence the
    list of binding dicts.
    """
    found: dict[str, list[dict[str, list[str]]]] = {}
    stack: list[tuple[str, dict[str, list[str]]]] = []
    for raw_line in text.splitlines():
        line = raw_line.split("//")[0].strip()
        if not line:
            continue
        if line.endswith("{"):
            stack.append((line[:-1].strip(), {}))
            continue
        if line == "}":
            if stack:
                name, binding = stack.pop()
                if binding:
                    found.setdefault(name, []).append(binding)
            continue
        if "=" not in line or ":" not in line or not stack:
            continue
        prop, _, value = line.partition("=")
        # Lines look like ``keyboardKey:i=56`` -> name "keyboardKey", type "i".
        prop_name, _, _prop_type = prop.partition(":")
        prop_name = prop_name.strip()
        if prop_name not in ("keyboardKey", "mouseButton", "joyButton"):
            continue
        stack[-1][1].setdefault(prop_name, []).append(value.strip())
    return found


def load_blk(path: str) -> dict[str, list[dict[str, list[str]]]]:
    if not path or not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8", errors="replace") as fh:
        return parse_blk(fh.read())


def _codes_from_binding(binding: dict[str, list[str]]) -> list[int]:
    """Keyboard scan code for one binding block.

    A block holding several keyboard keys is a modifier combo (Alt+Ctrl, say),
    which a single-key autopilot cannot reproduce, so it is skipped rather than
    half-pressed.
    """
    raw = binding.get("keyboardKey", [])
    if len(raw) != 1:
        return []
    try:
        return [int(raw[0])]
    except ValueError:
        return []


# Scan-code set 1: 59-68 are F1-F10 and 87-88 are F11-F12.  The range is not
# contiguous, and 71-83 in between are the numpad keys.
FUNCTION_KEY_CODES = frozenset(range(59, 69)) | {87, 88}


def _rank(code: int) -> tuple[bool, bool, int]:
    """Ordering used when one action has several candidate bindings.

    Function keys are ranked last: in the reference ``key.blk`` the throttle
    axis is bound to F1/F2 as well as to the numpad, and an F-key can trip an
    unrelated binding in game.  Non-extended codes beat extended ones, since an
    extended numpad code is read differently depending on NumLock.
    """
    return (code in FUNCTION_KEY_CODES, code >= 128, code)


def _collect(blk_bindings: dict[str, list[dict[str, list[str]]]]) -> dict[str, list[int]]:
    out: dict[str, list[int]] = {}
    for block_name, bindings in blk_bindings.items():
        action = AXIS_ACTIONS.get(block_name)
        if action is None:
            continue
        key = f"{action[0]}_{action[1]}"
        for binding in bindings:
            out.setdefault(key, []).extend(_codes_from_binding(binding))
    return out


def candidates(blk_bindings: dict[str, list[dict[str, list[str]]]]) -> dict[str, list[int]]:
    """Every code the control file offers for an action, for display in the UI."""
    return _collect(blk_bindings)


def resolve_bindings(
    blk_bindings: dict[str, list[dict[str, list[str]]]],
    overrides: dict[str, list[str]] | None = None,
) -> dict[str, list[int]]:
    """Map every autopilot action to the one scan code that will be sent.

    Priority: explicit user override > binding found in ``key.blk`` > fallback.
    Exactly one key per action, because holding several alternatives at once is
    not something the game would read the way the user intends.
    """
    overrides = overrides or {}
    from_blk = _collect(blk_bindings)
    resolved: dict[str, list[int]] = {}

    for action in ACTIONS:
        chosen: list[int] = []
        for name in overrides.get(action, []):
            code = name_to_code(name)
            if code is not None:
                chosen = [code]
                break
        if not chosen and from_blk.get(action):
            chosen = [min(from_blk[action], key=_rank)]
        if not chosen:
            chosen = [NAME_SCANCODES[n] for n in FALLBACKS.get(action, [])
                      if n in NAME_SCANCODES][:1]
        resolved[action] = chosen
    return resolved


def code_to_name(code: int) -> str:
    name = SCANCODE_NAMES.get(code & 0x7F, f"?{code}")
    return f"{name}{EXT_SUFFIX}" if code >= 128 else name


def describe(bindings: dict[str, list[int]]) -> dict[str, str]:
    return {action: ", ".join(code_to_name(c) for c in codes) or "-"
            for action, codes in bindings.items()}


def first_codes(bindings: dict[str, list[int]], actions: Iterable[str]) -> list[int]:
    """First bound code for each requested action, skipping unbound ones."""
    out = []
    for action in actions:
        codes = bindings.get(action) or []
        if codes:
            out.append(codes[0])
    return out