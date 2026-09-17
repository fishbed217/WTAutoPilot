"""Control laws: turn a :class:`~wtpilot.telemetry.Frame` into axis commands.

Two strategies govern the **heading channel** only (heading error -> bank):

1. **pd** - Continuous PD control with rate damping.
   Used when `control_strategy = "pd"`.
   Gains are constant; smooth but needs tuning.

2. **segmented** - Aokana-style segmented control with gain scheduling.
   Used when `control_strategy = "segmented"`.
   Coarse errors get aggressive response; fine errors get gentle response.
   More intuitive tuning, less prone to oscillation.

The **altitude channel** uses one law regardless of strategy; see the
``ALT_FAR_BAND_M`` constants below for the two-segment rule.

Everything downstream works in a conventional frame: positive roll means banked
right, positive pitch means nose up, positive aileron rolls right, positive
elevator pitches the nose up.  Calibration maps the raw instruments onto it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Literal

from .config import Params
from .nav import wrap180


class Mode(str, Enum):
    OFF = "off"
    STABILIZE = "stabilize"
    ALT_HOLD = "alt_hold"
    NAVIGATE = "navigate"

    @classmethod
    def parse(cls, value: str | None) -> "Mode":
        if not value:
            return cls.OFF
        try:
            return cls(str(value).lower())
        except ValueError:
            return cls.OFF


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def sign(x: float) -> int:
    return 1 if x > 0 else -1 if x < 0 else 0


# ---------------------------------------------------------------------------
# Segmented heading zones (inspired by Aokana-Flying-Circus)
# ---------------------------------------------------------------------------

@dataclass
class HeadingZone:
    """Control zone for heading error.

    Aokana uses discrete zones like:
        |error| > 15°  -> aggressive turn (bank toward max)
        |error| > 7°   -> medium turn
        |error| > 3°   -> gentle turn
        |error| > 0.3° -> micro correction
        else           -> dead zone (no input)

    Each zone maps to a target bank angle.
    """
    threshold_deg: float      # |heading error| threshold for this zone
    bank_deg: float           # target bank angle for this zone (absolute value)

    @classmethod
    def select(cls, zones: list["HeadingZone"], error_deg: float) -> float:
        """Select the appropriate bank angle for the given error.

        Returns target bank angle with sign matching the error direction.
        """
        abs_err = abs(error_deg)
        # Sort by threshold descending (largest first)
        for zone in sorted(zones, key=lambda z: z.threshold_deg, reverse=True):
            if abs_err >= zone.threshold_deg:
                return zone.bank_deg * sign(error_deg)
        return 0.0


# ---------------------------------------------------------------------------
# Altitude -> pitch law
# ---------------------------------------------------------------------------
#
# Two segments, hard-coded (not tunable from the UI):
#
#   |alt error| > ALT_FAR_BAND_M : hold a fixed ALT_FAR_PITCH_DEG attitude.
#       Deliberately NOT proportional to the error and deliberately without
#       vario damping - "hold an attitude" is the whole point.  The cost is
#       that the altitude loop is open-loop in attitude while in this band,
#       so the phugoid is undamped until the aircraft enters the near band.
#
#   |alt error| <= ALT_FAR_BAND_M : PD on the error, damped by the climb rate.
#
# The switch between the two can jump by up to ALT_FAR_PITCH_DEG +
# max_pitch_deg, so the result is always passed through a slew limiter.

ALT_FAR_BAND_M = 200.0        # |altitude error| above this holds a fixed pitch
ALT_FAR_PITCH_DEG = 7.0       # the fixed pitch held in the far band
PITCH_SLEW_DEG_PER_S = 10.0   # max change of pitch_target per second


# ---------------------------------------------------------------------------
# Default zone configurations (tuned for typical jet fighters)
# ---------------------------------------------------------------------------

DEFAULT_HEADING_ZONES = [
    HeadingZone(threshold_deg=15.0, bank_deg=35.0),   # Large error: aggressive turn
    HeadingZone(threshold_deg=7.0,  bank_deg=20.0),   # Medium error: moderate turn
    HeadingZone(threshold_deg=3.0,  bank_deg=10.0),   # Small error: gentle turn
    HeadingZone(threshold_deg=0.5,  bank_deg=3.0),    # Micro error: tiny correction
]


# ---------------------------------------------------------------------------
# Command dataclass
# ---------------------------------------------------------------------------

@dataclass
class Command:
    """Axis demands in [-1, 1]; see module docstring for sign conventions."""

    aileron: float = 0.0    # + rolls right
    elevator: float = 0.0   # + pitches nose up
    rudder: float = 0.0     # + yaws right
    throttle: float = 0.0   # + increases throttle

    roll_target: float = 0.0
    pitch_target: float = 0.0
    heading_target: float | None = None
    heading_error: float | None = None
    altitude_target: float | None = None
    altitude_error: float | None = None
    nav_bearing: float | None = None
    nav_distance_m: float | None = None
    arrived: bool = False
    notes: list[str] = field(default_factory=list)

    # For debugging: which zone was active
    heading_zone: str = ""
    altitude_zone: str = ""


# ---------------------------------------------------------------------------
# Flight Controller
# ---------------------------------------------------------------------------

class FlightController:
    """Stateful across ticks: holds hold-targets and rate estimates.

    ``control_strategy`` selects the heading channel only ("pd" or
    "segmented"); the altitude channel always uses the two-segment law.
    """

    def __init__(self, params: Params):
        self.params = params
        self.calibration = None  # set by the autopilot; needs roll/pitch signs
        self.reset()

    # ---- lifecycle ---------------------------------------------------------
    def reset(self) -> None:
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._prev_roll: float | None = None
        self._prev_pitch: float | None = None
        self._hold_heading: float | None = None
        self._hold_altitude: float | None = None
        self._pitch_target_prev: float | None = None
        # Custom zone configuration (if provided via params)
        self._heading_zones: list[HeadingZone] = []

    def capture_hold_targets(self, frame) -> None:
        """Freeze the current heading/altitude as the targets for this engagement."""
        self._hold_heading = frame.heading_deg
        self._hold_altitude = frame.altitude_m
        if self.params.target_altitude_m is None and frame.altitude_m is not None:
            self.params.target_altitude_m = frame.altitude_m

    @property
    def hold_heading(self) -> float | None:
        return self._hold_heading

    @property
    def hold_altitude(self) -> float | None:
        return self._hold_altitude

    # ---- per-tick ----------------------------------------------------------
    def update(self, frame, mode: Mode, dt: float) -> Command:
        cmd = Command()
        if not frame.valid or mode is Mode.OFF:
            self.reset()
            return cmd

        roll_sign = getattr(self.calibration, "roll_sign", 1)
        pitch_sign = getattr(self.calibration, "pitch_sign", 1)

        roll_raw = frame.roll_raw_deg
        pitch_raw = frame.pitch_raw_deg
        if roll_raw is None or pitch_raw is None:
            cmd.notes.append("姿态数据缺失")
            return cmd

        roll = roll_sign * roll_raw
        pitch = pitch_sign * pitch_raw
        self._update_rates(roll, pitch, dt)

        heading = frame.heading_deg
        altitude = frame.altitude_m
        vario = frame.vario_ms or 0.0

        # Navigate holds the altitude it had when the mode started.  Engaging
        # normally captures it, but the first valid reading may only arrive on a
        # later tick, so fall back to capturing here.
        if self._hold_altitude is None and altitude is not None:
            self._hold_altitude = altitude

        # Determine control strategy
        strategy = getattr(self.params, "control_strategy", "pd")
        if strategy not in ("pd", "segmented"):
            strategy = "pd"

        # --- altitude -> pitch ---------------------------------------------
        # The configured target altitude belongs to alt-hold alone; navigate
        # holds whatever altitude it had when it was engaged.
        if mode is Mode.STABILIZE:
            self._set_pitch_target(cmd, 0.0, dt)
        elif mode is Mode.ALT_HOLD:
            self._compute_pitch(cmd, altitude, vario, dt,
                                self.params.target_altitude_m)
        else:
            self._compute_pitch(cmd, altitude, vario, dt,
                                self._hold_altitude)

        # --- heading -> bank -----------------------------------------------
        needs_heading = mode in (Mode.ALT_HOLD, Mode.NAVIGATE)
        heading_error: float | None = None
        if mode is Mode.STABILIZE:
            cmd.roll_target = 0.0
        elif needs_heading:
            self._compute_roll(cmd, frame, heading, needs_heading, strategy)

        # --- attitude -> surfaces ------------------------------------------
        # Apply rate damping (always, helps prevent oscillation)
        cmd.aileron = clamp(
            self.params.roll_kp * (cmd.roll_target - roll)
            - self.params.roll_kd * self._roll_rate,
            1.0,
        )
        cmd.elevator = clamp(
            self.params.pitch_kp * (cmd.pitch_target - pitch)
            - self.params.pitch_kd * self._pitch_rate,
            1.0,
        )

        if self.params.use_rudder and cmd.heading_error is not None:
            cmd.rudder = clamp(self.params.rudder_kp * cmd.heading_error, 1.0)

        if self.params.use_throttle:
            cmd.throttle = self._throttle_demand(frame)

        return cmd

    # ---- strategy-specific computations ------------------------------------
    def _set_pitch_target(self, cmd: Command, raw: float, dt: float) -> None:
        """Clamp and slew-limit the pitch target, then record it on the command.

        The slew limiter rounds off the step at the far/near band boundary.
        The very first call after a reset passes through untouched so that
        engaging does not ramp up from an arbitrary seed.
        """
        raw = clamp(raw, self.params.max_pitch_deg)
        prev = self._pitch_target_prev
        if prev is None or dt <= 0:
            target = raw
        else:
            step = PITCH_SLEW_DEG_PER_S * dt
            target = prev + clamp(raw - prev, step)
        self._pitch_target_prev = target
        cmd.pitch_target = target

    def _compute_pitch(self, cmd: Command, altitude: float | None,
                       vario: float, dt: float,
                       target_alt: float | None) -> None:
        """Compute target pitch from altitude error."""
        cmd.altitude_target = target_alt

        if altitude is None or target_alt is None:
            self._set_pitch_target(cmd, 0.0, dt)
            return

        alt_error = target_alt - altitude
        cmd.altitude_error = alt_error

        if abs(alt_error) > ALT_FAR_BAND_M:
            cmd.altitude_zone = f"far: err={alt_error:.0f}m → {ALT_FAR_PITCH_DEG:.1f}°"
            self._set_pitch_target(cmd, ALT_FAR_PITCH_DEG * sign(alt_error), dt)
        else:
            raw = self.params.alt_kp * alt_error - self.params.vario_kd * vario
            cmd.altitude_zone = f"pd: err={alt_error:.0f}m, vy={vario:.1f}"
            self._set_pitch_target(cmd, raw, dt)

    def _compute_roll(self, cmd: Command, frame, heading: float | None,
                      needs_heading: bool, strategy: str) -> None:
        """Compute target roll from heading error."""
        heading_error: float | None = None

        # For navigate mode, use nav-provided heading error
        if frame.nav.get("heading_error") is not None:
            heading_error = frame.nav.get("heading_error")
            cmd.nav_bearing = frame.nav.get("bearing")
            cmd.nav_distance_m = frame.nav.get("distance_m")
            cmd.arrived = bool(frame.nav.get("arrived"))

        # Otherwise, compute from heading target
        if heading_error is None:
            target_heading = self._resolve_heading_target(heading)
            if heading is not None and target_heading is not None:
                heading_error = wrap180(target_heading - heading)

        cmd.heading_target = self.params.target_heading_deg or self._hold_heading

        if heading_error is None:
            cmd.roll_target = 0.0
            return

        cmd.heading_error = heading_error

        if strategy == "segmented":
            # Use zone-based bank selection
            zones = self._get_heading_zones()
            bank = HeadingZone.select(zones, heading_error)
            cmd.roll_target = clamp(bank, self.params.max_bank_deg)
            cmd.heading_zone = f"err={heading_error:.1f}° → bank={cmd.roll_target:.1f}°"
        else:
            # Original PD control
            cmd.roll_target = clamp(
                self.params.heading_kp * heading_error, self.params.max_bank_deg
            )

    def _get_heading_zones(self) -> list[HeadingZone]:
        """Get heading zones, allowing custom configuration."""
        if self._heading_zones:
            return self._heading_zones
        return DEFAULT_HEADING_ZONES

    # ---- helpers -----------------------------------------------------------
    def _resolve_heading_target(self, heading: float | None) -> float | None:
        if self.params.target_heading_deg is not None:
            return self.params.target_heading_deg
        return self._hold_heading

    def _throttle_demand(self, frame) -> float:
        ias = frame.ias_kmh
        if ias is None:
            return 0.0
        error = self.params.target_speed_kmh - ias
        return clamp(self.params.throttle_kp * error, 1.0)

    def _update_rates(self, roll: float, pitch: float, dt: float) -> None:
        if dt <= 0:
            return
        if self._prev_roll is not None:
            self._roll_rate = (roll - self._prev_roll) / dt
        if self._prev_pitch is not None:
            self._pitch_rate = (pitch - self._prev_pitch) / dt
        self._prev_roll, self._prev_pitch = roll, pitch

    def set_heading_zones(self, zones: list[HeadingZone]) -> None:
        """Set custom heading zones (for advanced tuning)."""
        self._heading_zones = zones


# ---------------------------------------------------------------------------
# Axis output shaping
# ---------------------------------------------------------------------------

def axis_state(value: float, deadband: float) -> int:
    """-1 / 0 / +1 for a normalized axis demand."""
    if not math.isfinite(value) or abs(value) < deadband:
        return 0
    return 1 if value > 0 else -1


class DitherAxis:
    """Turns a normalized demand into dithered key pulses.

    War Thunder's keyboard axes ramp toward full deflection while a key is held,
    so holding a key for a fraction of the ticks produces a proportional average
    deflection.  The accumulator below does that without breaking the loop's
    timing (no sleeps inside a tick), and several axes can be driven at once.
    """

    def __init__(self, deadband: float = 0.06):
        self.deadband = deadband
        self._acc = 0.0

    def reset(self) -> None:
        self._acc = 0.0

    def step(self, value: float) -> int:
        state = axis_state(value, self.deadband)
        if state == 0:
            self._acc = max(0.0, self._acc - 0.5)
            return 0
        duty = min(1.0, (abs(value) - self.deadband) / max(1e-6, 1.0 - self.deadband))
        self._acc += duty
        if self._acc >= 1.0:
            self._acc -= 1.0
            return state
        return 0