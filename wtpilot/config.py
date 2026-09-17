"""Runtime configuration: data source, control parameters, calibration, key overrides."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field, fields
from typing import Any

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF_DIR = os.path.join(PROJECT_ROOT, "ref")
CONFIG_DIR = os.path.join(PROJECT_ROOT, "config")
CONFIG_PATH = os.path.join(CONFIG_DIR, "config.json")
DEFAULT_BLK = os.path.join(REF_DIR, "key.blk")
UPLOADED_BLK = os.path.join(CONFIG_DIR, "controls.blk")

# Mode identifiers shared by the controller, the autopilot and the web UI.
MODES = ("stabilize", "alt_hold", "navigate")


@dataclass
class Settings:
    """Where telemetry comes from and how keys are delivered."""

    source: str = "http"  # "http" (localhost:8111), "powerbroker" (WarThunder package), or "sim"
    base_url: str = "http://localhost:8111"
    timeout_s: float = 0.4
    blk_path: str = DEFAULT_BLK
    # Refuse to emit keystrokes unless the game window is focused, so a held
    # control key can never leak into whatever the user is actually typing in.
    require_focus: bool = True
    focus_title: str = "War Thunder"
    autoconnect: bool = True
    # Fast-forward factor for the offline simulator only (1.0 = real time).
    sim_time_scale: float = 1.0
    # Never press keys, just compute and report the commands.
    dry_run: bool = False


@dataclass
class Params:
    """Control law tuning. Everything here is editable live from the web UI."""

    tick_hz: float = 20.0

    # Attitude limits.
    max_bank_deg: float = 45.0
    max_pitch_deg: float = 20.0

    # Roll channel: normalized aileron per degree of bank error.
    roll_kp: float = 0.040
    roll_kd: float = 0.0030

    # Pitch channel: normalized elevator per degree of pitch error.
    pitch_kp: float = 0.080
    pitch_kd: float = 0.0100

    # Heading -> bank (degrees of bank per degree of heading error).
    heading_kp: float = 1.20

    # Altitude -> pitch (degrees of pitch per metre of altitude error).
    alt_kp: float = 0.020
    vario_kd: float = 0.250

    # Targets. target_heading_deg None means "hold heading at engage time".
    target_altitude_m: float = 1000.0
    target_heading_deg: float | None = None
    arrive_radius_m: float = 400.0

    # Axis output shaping.
    axis_deadband: float = 0.06

    # Safety.
    min_altitude_m: float = 150.0

    # Throttle is optional: altitude is normally flown with pitch.
    use_throttle: bool = False
    throttle_target: float = 0.85
    throttle_kp: float = 0.004  # normalized throttle per m/s of speed error
    target_speed_kmh: float = 500.0

    # Rudder is not bound in the reference key.blk; off by default.
    use_rudder: bool = False
    rudder_kp: float = 0.010

    # Control strategy: "pd" (continuous PD) or "segmented" (Aokana-style zones).
    # Segmented control is more intuitive to tune; PD is smoother but needs
    # precise gain calibration.
    control_strategy: str = "pd"


@dataclass
class Calibration:
    """Signs for instrument readings.

    roll_sign / pitch_sign map the raw instrument reading onto a conventional
    frame: positive roll = banked right, positive pitch = nose up.

    War Thunder's aviahorizon_roll and aviahorizon_pitch are both negated
    relative to the standard convention, so the default signs are -1.
    The map_sign is learned passively from navigation.
    """

    roll_sign: int = -1
    pitch_sign: int = -1
    map_sign: int = 1


@dataclass
class Config:
    settings: Settings = field(default_factory=Settings)
    params: Params = field(default_factory=Params)
    calibration: Calibration = field(default_factory=Calibration)
    # action name -> list of key names (e.g. {"elevator_up": ["S"]})
    key_overrides: dict[str, list[str]] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Config":
        cfg = cls()
        for name, target in (
            ("settings", cfg.settings),
            ("params", cfg.params),
            ("calibration", cfg.calibration),
        ):
            section = data.get(name) or {}
            known = {f.name for f in fields(target)}
            for key, value in section.items():
                if key in known:
                    setattr(target, key, value)
        overrides = data.get("key_overrides") or {}
        cfg.key_overrides = {
            str(k): [str(v) for v in vals] for k, vals in overrides.items() if vals
        }
        return cfg

    def save(self, path: str = CONFIG_PATH) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.to_dict(), fh, indent=2)

    @classmethod
    def load(cls, path: str = CONFIG_PATH) -> "Config":
        if not os.path.exists(path):
            return cls()
        try:
            with open(path, encoding="utf-8") as fh:
                return cls.from_dict(json.load(fh))
        except (OSError, ValueError):
            # A corrupt config must never stop the app from starting.
            return cls()