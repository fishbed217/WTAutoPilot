"""Keyboard injection back ends.

War Thunder reads DirectInput, so strokes are sent as hardware scan codes via
``SendInput``.  ``KEYEVENTF_EXTENDEDKEY`` is set for scan codes with bit 7 set,
matching how the game encodes extended keys in its ``.blk`` files.
"""

from __future__ import annotations

import ctypes
import sys
import time
from ctypes import wintypes

KEYEVENTF_EXTENDEDKEY = 0x0001
KEYEVENTF_KEYUP = 0x0002
KEYEVENTF_SCANCODE = 0x0008
INPUT_KEYBOARD = 1

IS_WINDOWS = sys.platform == "win32"

if IS_WINDOWS:
    ULONG_PTR = ctypes.c_ulonglong if ctypes.sizeof(ctypes.c_void_p) == 8 else ctypes.c_ulong

    class KEYBDINPUT(ctypes.Structure):
        _fields_ = [
            ("wVk", wintypes.WORD),
            ("wScan", wintypes.WORD),
            ("dwFlags", wintypes.DWORD),
            ("time", wintypes.DWORD),
            ("dwExtraInfo", ULONG_PTR),
        ]

    class MOUSEINPUT(ctypes.Structure):
        _fields_ = [
            ("dx", wintypes.LONG),
            ("dy", wintypes.LONG),
            ("mouseData", wintypes.DWORD),
            ("dwFlags", wintypes.DWORD),
            ("time", wintypes.DWORD),
            ("dwExtraInfo", ULONG_PTR),
        ]

    class HARDWAREINPUT(ctypes.Structure):
        _fields_ = [
            ("uMsg", wintypes.DWORD),
            ("wParamL", wintypes.WORD),
            ("wParamH", wintypes.WORD),
        ]

    class _INPUTUNION(ctypes.Union):
        _fields_ = [("ki", KEYBDINPUT), ("mi", MOUSEINPUT), ("hi", HARDWAREINPUT)]

    class INPUT(ctypes.Structure):
        _anonymous_ = ("u",)
        _fields_ = [("type", wintypes.DWORD), ("u", _INPUTUNION)]


class KeySink:
    """Base class: press/release a key by raw scan code and release them all."""

    def press(self, code: int) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def release(self, code: int) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def release_all(self) -> None:
        for code in list(self.held):
            self.release(code)

    @property
    def held(self) -> set[int]:
        return getattr(self, "_held", set())

    def _mark(self, code: int, down: bool) -> None:
        if not hasattr(self, "_held"):
            self._held: set[int] = set()
        if down:
            self._held.add(code)
        else:
            self._held.discard(code)

    def close(self) -> None:
        self.release_all()


class NullSink(KeySink):
    """Dry run: records strokes, emits nothing.  Used for logging and tests."""

    def __init__(self) -> None:
        self._held: set[int] = set()
        self.log: list[tuple[float, str, int]] = []

    def press(self, code: int) -> None:
        if code in self._held:
            return
        self._mark(code, True)
        self.log.append((time.time(), "down", code))

    def release(self, code: int) -> None:
        if code not in self._held:
            return
        self._mark(code, False)
        self.log.append((time.time(), "up", code))


class SendInputSink(KeySink):
    """Real keyboard injection through ``user32.SendInput``."""

    def __init__(self) -> None:
        if not IS_WINDOWS:
            raise RuntimeError("SendInputSink is only available on Windows")
        self._held: set[int] = set()
        self._user32 = ctypes.WinDLL("user32", use_last_error=True)
        self._send = self._user32.SendInput
        self._send.argtypes = (wintypes.UINT, ctypes.POINTER(INPUT), ctypes.c_int)
        self._send.restype = wintypes.UINT

    def _emit(self, code: int, key_up: bool) -> None:
        flags = KEYEVENTF_SCANCODE
        scan = code
        if code >= 128:
            scan = code & 0x7F
            flags |= KEYEVENTF_EXTENDEDKEY
        if key_up:
            flags |= KEYEVENTF_KEYUP
        event = INPUT(type=INPUT_KEYBOARD)
        event.ki = KEYBDINPUT(wVk=0, wScan=scan, dwFlags=flags, time=0, dwExtraInfo=0)
        if self._send(1, ctypes.byref(event), ctypes.sizeof(INPUT)) != 1:
            raise OSError(f"SendInput failed: {ctypes.get_last_error()}")

    def press(self, code: int) -> None:
        if code in self._held:
            return
        self._emit(code, key_up=False)
        self._mark(code, True)

    def release(self, code: int) -> None:
        if code not in self._held:
            return
        self._emit(code, key_up=True)
        self._mark(code, False)


_USER32 = None


def _user32():
    global _USER32
    if _USER32 is None and IS_WINDOWS:
        _USER32 = ctypes.WinDLL("user32", use_last_error=True)
    return _USER32


def foreground_window_title() -> str:
    if not IS_WINDOWS:
        return ""
    user32 = _user32()
    hwnd = user32.GetForegroundWindow()
    if not hwnd:
        return ""
    length = user32.GetWindowTextLengthW(hwnd)
    buf = ctypes.create_unicode_buffer(length + 2)
    user32.GetWindowTextW(hwnd, buf, length + 2)
    return buf.value


def is_focused(title_substring: str) -> bool:
    """True when the foreground window title contains ``title_substring``.

    An empty filter means "do not check", which is what non-Windows callers get.
    """
    if not title_substring:
        return True
    if not IS_WINDOWS:
        return False
    return title_substring.lower() in foreground_window_title().lower()


def focus_window(title_substring: str) -> bool:
    """Bring the game window to the foreground.  Returns True on success."""
    if not IS_WINDOWS or not title_substring:
        return False
    user32 = _user32()
    found: list[int] = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def _enum(hwnd, _lparam):
        length = user32.GetWindowTextLengthW(hwnd)
        if length:
            buf = ctypes.create_unicode_buffer(length + 2)
            user32.GetWindowTextW(hwnd, buf, length + 2)
            if title_substring.lower() in buf.value.lower() and user32.IsWindowVisible(hwnd):
                found.append(hwnd)
                return False
        return True

    user32.EnumWindows(_enum, 0)
    if not found:
        return False
    hwnd = found[0]
    user32.ShowWindow(hwnd, 9)  # SW_RESTORE
    return bool(user32.SetForegroundWindow(hwnd))


def make_sink(dry_run: bool = False) -> KeySink:
    if dry_run or not IS_WINDOWS:
        return NullSink()
    try:
        return SendInputSink()
    except Exception:  # pragma: no cover - only on a broken win32 setup
        return NullSink()