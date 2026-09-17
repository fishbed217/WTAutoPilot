"""A small fixed-wing simulator that speaks the same JSON as the game.

obj.md rules out poking port 8111 while developing, so this stands in for War
Thunder: it emits ``Frame`` objects built from the same field names the game
uses, and accepts axis commands from the controller.  It exists to validate the
control laws and the web UI end to end.
"""

from __future__ import annotations

import math
import random
import time

from .nav import wrap180
from .telemetry import Frame, MapObject

G_MPS2 = 9.81
MAP_MIN = -65536.0
MAP_MAX = 65536.0
SPAN_M = MAP_MAX - MAP_MIN

# Internal map frame is rotated relative to true north and y points "down", so
# map angles run opposite to compass headings.  The controller is expected to
# discover this on its own via SignCalibrator.
MAP_ROTATION_DEG = 217.5


class FlightSim:
    """Kinematic model, good enough to exercise the controller, not a flight model."""

    def __init__(
        self,
        seed: int = 1234,
        roll_instr_sign: int = -1,
        pitch_instr_sign: int = -1,
        noise_deg: float = 0.05,
        start_alt_m: float = 900.0,
        start_heading_deg: float = 20.0,
        start_ias_ms: float = 190.0,
        start_xy: tuple[float, float] = (0.30, 0.70),
        poi_xy: tuple[float, float] = (0.75, 0.30),
    ) -> None:
        self._rng = random.Random(seed)
        self.roll_instr_sign = roll_instr_sign
        self.pitch_instr_sign = pitch_instr_sign
        self.noise_deg = noise_deg

        self.altitude = start_alt_m
        self.heading = start_heading_deg
        self.roll = 0.0        # conventional: + banked right
        self.pitch = 0.0       # conventional: + nose up
        self.ias_ms = start_ias_ms
        self.throttle = 0.85
        self.x, self.y = start_xy
        self.poi = poi_xy

        self._aileron = 0.0
        self._elevator = 0.0
        self.t = 0.0

    def close(self) -> None:
        return None

    # ---- input -------------------------------------------------------------
    def apply(self, command, dt: float) -> None:
        self._aileron = getattr(command, "aileron", 0.0)
        self._elevator = getattr(command, "elevator", 0.0)
        if getattr(command, "throttle", 0.0):
            self.throttle = max(0.0, min(1.0, self.throttle + command.throttle * dt * 0.8))
        self.step(dt)

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        self.t += dt

        # Attitude follows the surfaces with a first-order lag.
        bank_max = 60.0
        self.roll += (self._aileron * bank_max - self.roll) * min(1.0, dt / 0.7)
        self.pitch += (self._elevator * 25.0 - self.pitch) * min(1.0, dt / 0.9)

        # Coordinated turn: heading rate from bank angle.
        speed = max(40.0, self.ias_ms)
        turn_rate = math.degrees(G_MPS2 * math.tan(math.radians(self.roll)) / speed)
        self.heading = (self.heading + turn_rate * dt) % 360.0

        # Energy: thrust minus drag, plus the gravity component along the path.
        thrust = self.throttle * 8.0
        drag = 0.00025 * speed * speed
        self.ias_ms = max(40.0, self.ias_ms + (thrust - drag - G_MPS2 * math.sin(math.radians(self.pitch))) * dt)

        # Position along the facing vector, in normalized map units.
        angle = self._map_angle()
        step_norm = self.ias_ms * dt / SPAN_M
        self.x += math.cos(math.radians(angle)) * step_norm
        self.y += math.sin(math.radians(angle)) * step_norm

        self.altitude += self.ias_ms * math.sin(math.radians(self.pitch)) * dt

    # ---- output ------------------------------------------------------------
    def _map_angle(self) -> float:
        return MAP_ROTATION_DEG - self.heading

    @property
    def vario(self) -> float:
        return self.ias_ms * math.sin(math.radians(self.pitch))

    def _noisy(self, value: float) -> float:
        if not self.noise_deg:
            return value
        return value + self._rng.gauss(0.0, self.noise_deg)

    def player_object(self) -> MapObject:
        angle = math.radians(self._map_angle())
        return MapObject(
            type="aircraft",
            icon="Player",
            x=self.x,
            y=self.y,
            dx=math.cos(angle),
            dy=math.sin(angle),
            color="#faC81E",
        )

    def read(self) -> Frame:
        vario = self.vario
        ias_kmh = self.ias_ms * 3.6
        objects = [
            self.player_object(),
            MapObject(type="point_of_interest", icon="point_of_interest",
                      x=self.poi[0], y=self.poi[1], color="#fa0C00"),
            MapObject(type="aircraft", icon="Fighter", x=self.poi[0] + 0.05,
                      y=self.poi[1] + 0.05, dx=1.0, dy=0.0, color="#f00C00"),
        ]
        frame = Frame(
            timestamp=time.time(),
            valid=True,
            state={
                "H, m": self.altitude,
                "IAS, km/h": ias_kmh,
                "TAS, km/h": ias_kmh,
                "Vy, m/s": vario,
                "M": self.ias_ms / 340.0,
                "throttle 1, %": self.throttle * 100.0,
            },
            indicators={
                "valid": True,
                "army": "air",
                "type": "su_25tm",
                "compass": self.heading,
                "aviahorizon_roll": self._noisy(self.roll_instr_sign * self.roll),
                "aviahorizon_pitch": self._noisy(self.pitch_instr_sign * self.pitch),
                "bank": self._noisy(self.roll_instr_sign * self.roll),
                "vario": vario,
                "speed": self.ias_ms,
                "altitude_hour": self.altitude,
                "radio_altitude": self.altitude,
                "throttle": self.throttle,
            },
            objects=objects,
            map_info={"map_min": [MAP_MIN, MAP_MIN], "map_max": [MAP_MAX, MAP_MAX]},
        )
        return frame

    # ---- test helpers ------------------------------------------------------
    def poi_distance_m(self) -> float:
        return math.hypot(self.poi[0] - self.x, self.poi[1] - self.y) * SPAN_M

    def bearing_to_poi_deg(self) -> float:
        dx = self.poi[0] - self.x
        dy = self.poi[1] - self.y
        angle = math.degrees(math.atan2(dy, dx))
        return (MAP_ROTATION_DEG - angle) % 360.0

    def heading_error_to_poi_deg(self) -> float:
        return wrap180(self.bearing_to_poi_deg() - self.heading)