"""Map geometry: where the point of interest is, relative to where we are pointing.

``map_obj.json`` gives normalized map coordinates and, for the player, a facing
vector ``(dx, dy)``.  That vector is the key: by measuring the *angle between*
our facing vector and the vector to the target, we get the turn required
without needing to know the map's absolute orientation or its units.

The map frame is rotated relative to true north (verified against the reference
capture: compass 187.9 deg corresponds to a facing vector of roughly 30 deg in
map space), and it may or may not be mirrored.  The sign of that relationship is
learned passively by watching how the facing angle moves while the compass
changes, so no assumption about the map's handedness is baked in.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence


def wrap180(angle: float) -> float:
    """Normalize to (-180, 180]."""
    return (angle + 180.0) % 360.0 - 180.0


def _angle_of(vx: float, vy: float) -> float | None:
    if vx == 0.0 and vy == 0.0:
        return None
    return math.degrees(math.atan2(vy, vx))


def turn_to_target(
    own_xy: Sequence[float],
    own_facing: Sequence[float] | None,
    target_xy: Sequence[float],
    sign: int = 1,
) -> tuple[float | None, float]:
    """Return ``(heading_error_deg, distance_in_map_units)``.

    ``heading_error_deg`` is positive when the target lies to the right, i.e.
    the aircraft must turn clockwise (compass increasing) to line up.
    """
    vx = target_xy[0] - own_xy[0]
    vy = target_xy[1] - own_xy[1]
    distance = math.hypot(vx, vy)

    target_angle = _angle_of(vx, vy)
    if target_angle is None:
        return 0.0, 0.0
    if own_facing is None:
        return None, distance
    own_angle = _angle_of(own_facing[0], own_facing[1])
    if own_angle is None:
        return None, distance

    return wrap180(sign * wrap180(target_angle - own_angle)), distance


def relative_bearing(
    own_xy: Sequence[float],
    own_facing: Sequence[float] | None,
    target_xy: Sequence[float],
    compass_deg: float | None,
    sign: int = 1,
) -> tuple[float | None, float | None, float]:
    """Return ``(target_bearing_deg, heading_error_deg, distance_units)``.

    The bearing is expressed on the compass, derived from the aircraft's own
    facing vector so map rotation cancels out.
    """
    error, distance = turn_to_target(own_xy, own_facing, target_xy, sign)
    if error is None:
        return None, None, distance
    if compass_deg is None:
        return None, error, distance
    bearer = (compass_deg + error) % 360.0
    return bearer, error, distance


class SignCalibrator:
    """Learns whether map-frame angles turn the same way as the compass.

    Change is accumulated from a reference sample until both the map angle and
    the compass have moved far enough to mean something, then the two are
    compared and one vote is cast.  Accumulating matters because a single
    control tick moves the heading by a fraction of a degree: comparing
    consecutive ticks would never clear the noise floor, while comparing over a
    whole turn gives a clear verdict.

    ``sign()`` returns ``None``-like default until enough votes accumulate.
    """

    MIN_VOTES = 2
    MIN_DELTA_DEG = 3.0
    MAX_DELTA_DEG = 90.0  # beyond this the wrap-around is ambiguous

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._ref: tuple[float, float] | None = None
        self.agree = 0
        self.disagree = 0

    def observe(self, own_facing: Sequence[float] | None, compass_deg: float | None) -> None:
        if own_facing is None or compass_deg is None:
            return
        angle = _angle_of(own_facing[0], own_facing[1])
        if angle is None:
            return
        if self._ref is None:
            self._ref = (angle, compass_deg)
            return

        d_map = wrap180(angle - self._ref[0])
        d_hdg = wrap180(compass_deg - self._ref[1])
        if abs(d_map) > self.MAX_DELTA_DEG or abs(d_hdg) > self.MAX_DELTA_DEG:
            self._ref = (angle, compass_deg)  # lost coherence, start over
            return
        if abs(d_map) < self.MIN_DELTA_DEG or abs(d_hdg) < self.MIN_DELTA_DEG:
            return  # not enough signal yet; keep accumulating

        if (d_map > 0) == (d_hdg > 0):
            self.agree += 1
        else:
            self.disagree += 1
        self._ref = (angle, compass_deg)

    @property
    def votes(self) -> int:
        return self.agree + self.disagree

    def sign(self, default: int = 1) -> int:
        if self.votes < self.MIN_VOTES:
            return default
        return 1 if self.agree >= self.disagree else -1


def nearest_object(
    objects: Iterable, predicate, own_xy: Sequence[float], meters_per_unit: float
) -> tuple[object | None, float | None]:
    best = None
    best_dist = None
    for obj in objects:
        if not predicate(obj):
            continue
        dist = math.hypot(obj.x - own_xy[0], obj.y - own_xy[1]) * meters_per_unit
        if best_dist is None or dist < best_dist:
            best, best_dist = obj, dist
    return best, best_dist