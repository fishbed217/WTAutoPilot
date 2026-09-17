"""Telemetry: read War Thunder's local HTTP API into a single :class:`Frame`.

Endpoints used (all served by the game on localhost:8111 while flying):

* ``/state``         - primary flight data (altitude, IAS, Vy, throttle, ...)
* ``/indicators``    - cockpit instruments (attitude, compass, vario, RPM, ...)
* ``/map_obj.json``  - map objects: the player, the point of interest, contacts
* ``/map_info.json`` - map bounds, used to turn normalized coords into metres
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any

import requests

DEFAULT_SPAN_M = 131072.0  # fallback map extent when /map_info.json is missing


def _f(value: Any, default: float | None = None) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


@dataclass
class MapObject:
    type: str
    icon: str
    x: float
    y: float
    dx: float | None = None
    dy: float | None = None
    color: str = ""

    @property
    def is_player(self) -> bool:
        return self.type == "aircraft" and self.icon.lower() == "player"

    @property
    def is_enemy_aircraft(self) -> bool:
        return self.type == "aircraft" and not self.is_player


@dataclass
class Frame:
    """One consistent snapshot of the aircraft and the world around it."""

    timestamp: float
    valid: bool = False
    error: str = ""
    state: dict[str, Any] = field(default_factory=dict)
    indicators: dict[str, Any] = field(default_factory=dict)
    objects: list[MapObject] = field(default_factory=list)
    map_info: dict[str, Any] = field(default_factory=dict)

    # Filled in by the autopilot before the controller sees the frame.
    nav: dict[str, Any] = field(default_factory=dict)

    # ---- raw-ish accessors -------------------------------------------------
    @property
    def altitude_m(self) -> float | None:
        for key in ("H, m",):
            if (v := _f(self.state.get(key))) is not None:
                return v
        for key in ("altitude_hour", "altitude1_min"):
            if (v := _f(self.indicators.get(key))) is not None:
                return v
        return _f(self.indicators.get("radio_altitude"))

    @property
    def radio_altitude_m(self) -> float | None:
        return _f(self.indicators.get("radio_altitude"))

    @property
    def ias_kmh(self) -> float | None:
        if (v := _f(self.state.get("IAS, km/h"))) is not None:
            return v
        if (v := _f(self.indicators.get("speed"))) is not None:
            return v * 3.6
        return None

    @property
    def tas_kmh(self) -> float | None:
        if (v := _f(self.state.get("TAS, km/h"))) is not None:
            return v
        speed = _f(self.indicators.get("speed"))
        return speed * 3.6 if speed is not None else None

    @property
    def heading_deg(self) -> float | None:
        return _f(self.indicators.get("compass"))

    @property
    def roll_raw_deg(self) -> float | None:
        """Raw instrument bank.  Positive direction is calibration dependent."""
        if (v := _f(self.indicators.get("aviahorizon_roll"))) is not None:
            return v
        return _f(self.indicators.get("bank"))

    @property
    def pitch_raw_deg(self) -> float | None:
        """Raw instrument pitch.  Positive direction is calibration dependent."""
        return _f(self.indicators.get("aviahorizon_pitch"))

    @property
    def vario_ms(self) -> float | None:
        if (v := _f(self.indicators.get("vario"))) is not None:
            return v
        return _f(self.state.get("Vy, m/s"))

    @property
    def throttle(self) -> float | None:
        if (v := _f(self.state.get("throttle 1, %"))) is not None:
            return v / 100.0
        return _f(self.indicators.get("throttle"))

    @property
    def player(self) -> MapObject | None:
        for obj in self.objects:
            if obj.is_player:
                return obj
        return None

    @property
    def point_of_interest(self) -> MapObject | None:
        for obj in self.objects:
            if obj.type == "point_of_interest":
                return obj
        return None

    @property
    def enemies(self) -> list[MapObject]:
        return [o for o in self.objects if o.is_enemy_aircraft]

    # ---- derived -----------------------------------------------------------
    @property
    def map_span_m(self) -> tuple[float, float]:
        try:
            mn = self.map_info["map_min"]
            mx = self.map_info["map_max"]
            sx = abs(float(mx[0]) - float(mn[0])) or DEFAULT_SPAN_M
            sy = abs(float(mx[1]) - float(mn[1])) or DEFAULT_SPAN_M
            return sx, sy
        except (KeyError, IndexError, TypeError, ValueError):
            return DEFAULT_SPAN_M, DEFAULT_SPAN_M

    @property
    def meters_per_map_unit(self) -> float:
        sx, sy = self.map_span_m
        return (sx + sy) / 2.0

    def summary(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "error": self.error,
            "altitude_m": self.altitude_m,
            "radio_altitude_m": self.radio_altitude_m,
            "ias_kmh": self.ias_kmh,
            "heading_deg": self.heading_deg,
            "vario_ms": self.vario_ms,
            "roll_raw_deg": self.roll_raw_deg,
            "pitch_raw_deg": self.pitch_raw_deg,
            "throttle": self.throttle,
            "position": [self.player.x, self.player.y] if self.player else None,
            "facing": [self.player.dx, self.player.dy] if self.player else None,
            "poi": [self.point_of_interest.x, self.point_of_interest.y]
            if self.point_of_interest
            else None,
            "enemy_count": len(self.enemies),
            "map_span_m": list(self.map_span_m),
        }


def _parse_objects(raw: Any) -> list[MapObject]:
    objects: list[MapObject] = []
    if not isinstance(raw, list):
        return objects
    for item in raw:
        if not isinstance(item, dict):
            continue
        try:
            x = float(item["x"])
            y = float(item["y"])
        except (KeyError, TypeError, ValueError):
            continue
        objects.append(
            MapObject(
                type=str(item.get("type", "")),
                icon=str(item.get("icon", "none")),
                x=x,
                y=y,
                dx=_f(item.get("dx")),
                dy=_f(item.get("dy")),
                color=str(item.get("color", "")),
            )
        )
    return objects


class HttpSource:
    """Polls the game's local HTTP API.

    Every endpoint is optional: a failed request degrades one field group rather
    than the whole frame, so a transient /map_obj.json hiccup does not drop the
    control loop.
    """

    def __init__(self, base_url: str = "http://localhost:8111", timeout_s: float = 0.4):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s
        self._session = requests.Session()
        self._last_objects: list[MapObject] = []
        self._last_map_info: dict[str, Any] = {}

    def _get_json(self, path: str) -> Any:
        resp = self._session.get(f"{self.base_url}{path}", timeout=self.timeout_s)
        resp.raise_for_status()
        return resp.json()

    def connect(self) -> None:
        """One-shot probe so the UI can report reachability."""

    def read(self) -> Frame:
        import time

        frame = Frame(timestamp=time.time())
        errors: list[str] = []

        try:
            frame.state = self._get_json("/state") or {}
        except Exception as exc:
            errors.append(f"state: {exc}")

        try:
            frame.indicators = self._get_json("/indicators") or {}
        except Exception as exc:
            errors.append(f"indicators: {exc}")

        try:
            self._last_objects = _parse_objects(self._get_json("/map_obj.json"))
            frame.objects = self._last_objects
        except Exception as exc:
            errors.append(f"map_obj: {exc}")
            frame.objects = self._last_objects

        try:
            self._last_map_info = self._get_json("/map_info.json") or {}
        except Exception:
            pass
        frame.map_info = self._last_map_info

        frame.valid = bool(frame.state or frame.indicators)
        frame.error = "; ".join(errors)
        return frame

    def apply(self, command: Any, dt: float) -> None:
        """No-op: the real game advances on its own.  Mirrors SimSource."""

    def close(self) -> None:
        self._session.close()


class PowerBrokerSource:
    """HttpSource alternative using the PowerBroker2/WarThunder PyPI package.

    Benefits:
    - Maintained by the community with known issues fixed
    - Handles altitude unit conversion (ft→m for US/UK planes)
    - Pre-flips aviahorizon_roll/pitch signs

    Install: pip install WarThunder
    """

    def __init__(self, base_url: str = "http://localhost:8111", timeout_s: float = 0.4):
        self.base_url = base_url
        self.timeout_s = timeout_s
        self._telem = None
        self._last_objects: list[MapObject] = []
        self._last_map_info: dict[str, Any] = {}

    def _init_telem(self) -> bool:
        """Lazily initialize the WarThunder.TelemInterface."""
        if self._telem is not None:
            return True
        try:
            from WarThunder import telemetry, mapinfo
            self._telem = telemetry.TelemInterface()
            self._mapinfo = mapinfo.MapInfo()
            return True
        except ImportError:
            return False

    def connect(self) -> None:
        pass

    def read(self) -> Frame:
        import time

        frame = Frame(timestamp=time.time())
        errors: list[str] = []

        if not self._init_telem():
            frame.error = "WarThunder package not installed"
            return frame

        try:
            # Fetch state + indicators
            self._telem.get_telemetry()
            frame.state = self._telem.state or {}
            frame.indicators = self._telem.indicators or {}

            # The library pre-flips roll/pitch signs, but our calibration
            # expects raw values. Put them back.
            if "aviahorizon_roll" in frame.indicators:
                frame.indicators["aviahorizon_roll"] = -frame.indicators["aviahorizon_roll"]
            if "aviahorizon_pitch" in frame.indicators:
                frame.indicators["aviahorizon_pitch"] = -frame.indicators["aviahorizon_pitch"]
        except Exception as exc:
            errors.append(f"telemetry: {exc}")

        # Map objects - the WarThunder library doesn't expose them directly,
        # so we use raw requests for now
        try:
            resp = requests.get(f"{self.base_url}/map_obj.json", timeout=self.timeout_s)
            self._last_objects = _parse_objects(resp.json())
            frame.objects = self._last_objects
        except Exception as exc:
            errors.append(f"map_obj: {exc}")
            frame.objects = self._last_objects

        try:
            resp = requests.get(f"{self.base_url}/map_info.json", timeout=self.timeout_s)
            self._last_map_info = resp.json() or {}
        except Exception:
            pass
        frame.map_info = self._last_map_info

        frame.valid = bool(frame.state or frame.indicators)
        frame.error = "; ".join(errors)
        return frame

    def apply(self, command: Any, dt: float) -> None:
        pass

    def close(self) -> None:
        pass


class FixtureSource:
    """Replays the JSON captured in ``ref/`` without touching the network.

    Useful for smoke-testing the plumbing and for the test suite.
    """

    def __init__(self, ref_dir: str):
        self.ref_dir = ref_dir

    def _load(self, name: str) -> Any:
        path = os.path.join(self.ref_dir, name)
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)

    def read(self) -> Frame:
        import time

        frame = Frame(timestamp=time.time(), valid=True)
        try:
            frame.state = self._load("state.json")
            frame.indicators = self._load("indicators.json")
            frame.objects = _parse_objects(self._load("map_obj.json"))
        except (OSError, ValueError) as exc:
            frame.valid = False
            frame.error = str(exc)
        return frame

    def apply(self, command: Any, dt: float) -> None:
        return None

    def close(self) -> None:
        return None