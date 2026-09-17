"""The autopilot: reads telemetry, runs the controller, drives the keyboard.

The tick body is a plain :meth:`Autopilot.tick` so tests can step it
deterministically; :meth:`Autopilot.start` just runs it on a timer.

``time_scale`` fast-forwards the offline simulator (see ``Settings.sim_time_scale``)
so a full flight can be watched in a few seconds.  It stays at 1.0 against the
real game, where wall-clock time is the only clock available.
"""

from __future__ import annotations

import threading
import time
from dataclasses import asdict

from . import keys as keys_mod
from .config import MODES, Config
from .control import Command, DitherAxis, FlightController, Mode
from .keymap import code_to_name, describe, load_blk, resolve_bindings
from .nav import SignCalibrator, relative_bearing

# Display names for the mode identifiers, which stay English in the API.
MODE_NAMES = {
    Mode.STABILIZE.value: "保持稳定",
    Mode.ALT_HOLD.value: "确定高度",
    Mode.NAVIGATE.value: "飞往目标点",
}


class Autopilot:
    def __init__(self, source, config: Config, sink=None, dry_run: bool = False):
        self.source = source
        self.config = config
        self.settings = config.settings
        self.params = config.params
        self.calibration = config.calibration

        self.controller = FlightController(self.params)
        self.controller.calibration = self.calibration
        self._sign_cal = SignCalibrator()

        self.dry_run = dry_run
        self.sink = sink if sink is not None else keys_mod.make_sink(dry_run=dry_run)
        self._injecting = not isinstance(self.sink, keys_mod.NullSink)

        # Only the simulator may be run faster than real time.
        self.time_scale = (
            max(1.0, self.settings.sim_time_scale) if self.settings.source == "sim" else 1.0
        )

        self.bindings = resolve_bindings(
            load_blk(self.settings.blk_path), config.key_overrides
        )
        self.axes = {
            name: DitherAxis(self.params.axis_deadband)
            for name in ("aileron", "elevator", "rudder", "throttle")
        }

        self._lock = threading.RLock()
        self.mode = Mode.OFF
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._phase = "idle"
        self._message = "就绪"

        self.frame = None
        self.command = Command()
        self.ticks = 0
        self.loop_hz = 0.0
        self._last_frame_error = ""

    # ---- configuration -----------------------------------------------------
    def reload_bindings(self, blk_path: str | None = None) -> dict[str, str]:
        with self._lock:
            if blk_path:
                self.settings.blk_path = blk_path
            self.bindings = resolve_bindings(
                load_blk(self.settings.blk_path), self.config.key_overrides
            )
            return describe(self.bindings)

    # The web UI's blank field means "unset" for these, so null is a real value
    # when saved via /api/params. However, during /api/engage, we want to
    # preserve the existing value if the field was blank.
    # This is handled by engage() not calling update_params with null targets.
    NULLABLE_PARAMS = ("target_altitude_m", "target_heading_deg")

    def update_params(self, values: dict, allow_null_targets: bool = False) -> None:
        """Update parameters from a dict.

        Args:
            values: Parameter key-value pairs
            allow_null_targets: If True, null target_altitude_m/target_heading_deg
                are accepted (clears them). If False (default), null targets are
                ignored, preserving existing values.
        """
        with self._lock:
            for key, value in values.items():
                if not hasattr(self.params, key):
                    continue
                if value is None and key not in self.NULLABLE_PARAMS:
                    continue
                if value is None and key in self.NULLABLE_PARAMS and not allow_null_targets:
                    continue
                setattr(self.params, key, value)
            for axis in self.axes.values():
                axis.deadband = self.params.axis_deadband

    def set_calibration(self, **values) -> None:
        with self._lock:
            for key, value in values.items():
                if hasattr(self.calibration, key):
                    setattr(self.calibration, key, int(value))

    def set_source(self, source, dry_run: bool | None = None) -> None:
        """Swap the telemetry source (live game <-> simulator) at runtime."""
        self.disengage("遥测数据源已切换")
        try:
            self.source.close()
        except Exception:
            pass
        if dry_run is not None and dry_run != self.dry_run:
            self.dry_run = dry_run
            self.sink.close()
            self.sink = keys_mod.make_sink(dry_run=dry_run)
            self._injecting = not isinstance(self.sink, keys_mod.NullSink)
        self.source = source
        self.time_scale = (
            max(1.0, self.settings.sim_time_scale) if self.settings.source == "sim" else 1.0
        )
        self._sign_cal.reset()
        self.frame = None

    # ---- lifecycle ---------------------------------------------------------
    def engage(self, mode: str | Mode) -> bool:
        parsed = Mode.parse(mode) if isinstance(mode, str) else mode
        if parsed not in (Mode.STABILIZE, Mode.ALT_HOLD, Mode.NAVIGATE):
            self._message = f"未知模式: {mode}"
            return False
        with self._lock:
            self.controller.reset()
            if self.frame is not None and self.frame.valid:
                self.controller.capture_hold_targets(self.frame)
            self.mode = parsed
            self._phase = "running"
            self._message = f"已接通: {MODE_NAMES.get(parsed.value, parsed.value)}"
        if not self._thread or not self._thread.is_alive():
            self.start()
        return True

    def disengage(self, message: str = "已脱开") -> None:
        with self._lock:
            self.mode = Mode.OFF
            self._phase = "idle"
            self._message = message
            self.controller.reset()
        self._release_all()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="wt-autopilot", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None

    def close(self) -> None:
        self.disengage("程序退出")
        self.stop()
        try:
            self.sink.close()
        finally:
            self.source.close()

    # ---- loop --------------------------------------------------------------
    def _run(self) -> None:
        tick = 1.0 / max(1.0, self.params.tick_hz)
        wall_tick = tick / self.time_scale
        last = time.perf_counter()
        while not self._stop.is_set():
            now = time.perf_counter()
            dt = now - last
            last = now
            try:
                self.tick(dt)
            except Exception as exc:  # never let the loop die silently
                self._release_all(f"循环异常: {exc}")
            self.loop_hz = 1.0 / max(1e-6, dt)
            self._stop.wait(wall_tick)

    def tick(self, dt: float) -> None:
        dt = max(1e-3, min(0.5, dt)) * self.time_scale
        frame = self.source.read()
        self.frame = frame
        self.ticks += 1
        if frame.error:
            self._last_frame_error = frame.error

        if not frame.valid:
            self._release_all(f"遥测中断（{frame.error or '无数据'}）")
            return

        self._update_nav(frame)

        with self._lock:
            mode = self.mode

        if mode is Mode.OFF:
            self.command = Command()
            return

        command = self.controller.update(frame, mode, dt)
        self.command = command

        if self._injecting and not self._focus_ok():
            self._release_all("游戏窗口不在前台")
            return

        self._apply_axes(command)
        self._check_safety(frame)

        try:
            self.source.apply(command, dt)
        except Exception as exc:
            self._release_all(f"模拟器步进异常: {exc}")

    # ---- pieces ------------------------------------------------------------
    def _update_nav(self, frame) -> None:
        frame.nav = {}
        player = frame.player
        if player is None:
            return
        facing = (player.dx, player.dy) if player.dx is not None else None
        self._sign_cal.observe(facing, frame.heading_deg)
        sign = self._sign_cal.sign(self.calibration.map_sign)
        if self._sign_cal.votes >= SignCalibrator.MIN_VOTES:
            self.calibration.map_sign = sign

        poi = frame.point_of_interest
        if poi is None:
            return
        bearing, error, distance_units = relative_bearing(
            (player.x, player.y), facing, (poi.x, poi.y), frame.heading_deg, sign
        )
        distance_m = distance_units * frame.meters_per_map_unit
        frame.nav = {
            "bearing": None if bearing is None else round(bearing, 1),
            "heading_error": None if error is None else round(error, 1),
            "distance_m": round(distance_m, 1),
            "arrived": distance_m <= self.params.arrive_radius_m,
            "map_sign": sign,
        }

    def _focus_ok(self) -> bool:
        if not self.settings.require_focus:
            return True
        return keys_mod.is_focused(self.settings.focus_title)

    def _apply_axes(self, command: Command) -> None:
        demands = {
            "aileron": (command.aileron, "aileron_left", "aileron_right"),
            "elevator": (command.elevator, "elevator_down", "elevator_up"),
            "rudder": (command.rudder, "rudder_left", "rudder_right"),
            "throttle": (command.throttle, "throttle_down", "throttle_up"),
        }
        for name, (value, neg_action, pos_action) in demands.items():
            neg_codes = self.bindings.get(neg_action) or []
            pos_codes = self.bindings.get(pos_action) or []
            if not self._axis_enabled(name):
                # Feature just turned off: make sure nothing is left held down.
                self.axes[name].reset()
                self._set_key(neg_codes, False)
                self._set_key(pos_codes, False)
                continue
            state = self.axes[name].step(value)
            self._set_key(neg_codes, state < 0)
            self._set_key(pos_codes, state > 0)

    def _axis_enabled(self, name: str) -> bool:
        if name == "rudder":
            return self.params.use_rudder
        if name == "throttle":
            return self.params.use_throttle
        return True

    def _set_key(self, codes: list[int], down: bool) -> None:
        for code in codes:
            if down:
                self.sink.press(code)
            else:
                self.sink.release(code)

    def _release_all(self, message: str | None = None) -> None:
        self.sink.release_all()
        for axis in self.axes.values():
            axis.reset()
        if message is not None:
            self._message = message
            self._phase = "waiting"

    def _check_safety(self, frame) -> None:
        if self.ticks < 5:  # ignore the first ticks, the reading may be stale
            return
        altitude = frame.altitude_m
        if altitude is not None and altitude < self.params.min_altitude_m:
            self.disengage(f"安全保护: 低于 {self.params.min_altitude_m:.0f} m")

    # ---- reporting ---------------------------------------------------------
    def status(self) -> dict:
        frame = self.frame
        with self._lock:
            mode = self.mode.value
            phase = self._phase
            message = self._message
        return {
            "engaged": mode != Mode.OFF.value,
            "mode": mode,
            "modes_available": list(MODES),
            "phase": phase,
            "message": message,
            "source": self.settings.source,
            "injecting_keys": self._injecting,
            "dry_run": self.dry_run,
            "focused": self._focus_ok() if self._injecting else True,
            "focus_title": self.settings.focus_title,
            "require_focus": self.settings.require_focus,
            "ticks": self.ticks,
            "loop_hz": round(self.loop_hz, 1),
            "time_scale": self.time_scale,
            "keys_held": [code_to_name(c) for c in sorted(self.sink.held)],
            "telemetry": frame.summary() if frame is not None else None,
            "nav": (frame.nav if frame is not None else {}) or {},
            "command": self._command_dict(),
            "calibration": asdict(self.calibration),
            "params": asdict(self.params),
            "bindings": describe(self.bindings),
            "last_error": self._last_frame_error,
        }

    def _command_dict(self) -> dict:
        cmd = self.command
        return {
            "aileron": round(cmd.aileron, 3),
            "elevator": round(cmd.elevator, 3),
            "rudder": round(cmd.rudder, 3),
            "throttle": round(cmd.throttle, 3),
            "roll_target": round(cmd.roll_target, 2),
            "pitch_target": round(cmd.pitch_target, 2),
            "heading_error": None if cmd.heading_error is None else round(cmd.heading_error, 1),
            "altitude_error": None if cmd.altitude_error is None else round(cmd.altitude_error, 1),
            "arrived": cmd.arrived,
        }