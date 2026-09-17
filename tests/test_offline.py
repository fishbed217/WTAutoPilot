"""Offline verification.

obj.md forbids poking port 8111 during development, so everything here runs
against ``ref/key.blk``, the recorded JSON in ``ref/`` and the bundled
simulator.  Nothing in this file opens a socket.
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wtpilot.autopilot import Autopilot
from wtpilot.config import REF_DIR, Config
from wtpilot.control import (
    ALT_FAR_BAND_M,
    ALT_FAR_PITCH_DEG,
    PITCH_SLEW_DEG_PER_S,
    Command,
    DitherAxis,
    FlightController,
    Mode,
    axis_state,
)
from wtpilot.keymap import (
    candidates,
    code_to_name,
    describe,
    load_blk,
    name_to_code,
    resolve_bindings,
)
from wtpilot.keys import NullSink
from wtpilot.nav import SignCalibrator, relative_bearing, turn_to_target, wrap180
from wtpilot.sim import FlightSim
from wtpilot.telemetry import FixtureSource

BLK_PATH = os.path.join(REF_DIR, "key.blk")


def set_calibration(config, roll=-1, pitch=-1, map_sign=1):
    config.calibration.roll_sign = roll
    config.calibration.pitch_sign = pitch
    config.calibration.map_sign = map_sign


class BlkParsingTest(unittest.TestCase):
    def setUp(self):
        self.blk = load_blk(BLK_PATH)

    def test_reads_hotkey_blocks(self):
        self.assertIn("elevator_rangeMax", self.blk)
        self.assertIn("ailerons_rangeMin", self.blk)
        self.assertIn("ID_AIR_BRAKE", self.blk)
        # Repeated bindings accumulate rather than overwrite.
        self.assertGreaterEqual(len(self.blk["ID_ROCKETS_SERIES"]), 2)

    def test_flight_axes_resolve_to_wsad(self):
        bindings = resolve_bindings(self.blk)
        # War Thunder's default keyboard flight axes.
        self.assertEqual(bindings["elevator_up"], [31])    # S
        self.assertEqual(bindings["elevator_down"], [17])  # W
        self.assertEqual(bindings["aileron_left"], [30])   # A
        self.assertEqual(bindings["aileron_right"], [32])  # D

    def test_uncoded_actions_fall_back(self):
        bindings = resolve_bindings(self.blk)
        # key.blk does not bind keyboard rudder, so the fallback applies and is
        # reported clearly rather than silently sending nothing.
        self.assertEqual(bindings["rudder_left"], [16])  # Q
        self.assertEqual(bindings["rudder_right"], [18])  # E

    def test_extended_numpad_bindings_are_kept_when_alone(self):
        blk = {"vtol_rangeMax": [{"keyboardKey": ["201"]}]}
        self.assertEqual(blk["vtol_rangeMax"][0]["keyboardKey"], ["201"])
        self.assertEqual(code_to_name(201), "NUMPAD9_EXT")

    def test_name_code_round_trip(self):
        for code in (17, 31, 30, 32, 59, 73, 201, 209, 210):
            self.assertEqual(name_to_code(code_to_name(code)), code)

    def test_every_action_resolves_to_at_most_one_key(self):
        bindings = resolve_bindings(self.blk)
        for action, codes in bindings.items():
            self.assertLessEqual(
                len(codes), 1, f"{action} would hold {len(codes)} keys at once"
            )

    def test_throttle_prefers_numpad_over_function_keys(self):
        # key.blk binds throttle_rangeMax to F1, NUMPAD9 and an extended NUMPAD9.
        bindings = resolve_bindings(self.blk)
        self.assertEqual(bindings["throttle_up"], [73])    # NUMPAD9, not F1
        self.assertEqual(bindings["throttle_down"], [77])  # NUMPAD6, not F2
        self.assertNotIn(59, bindings["throttle_up"])
        self.assertNotIn(60, bindings["throttle_down"])

    def test_all_candidates_are_still_reported(self):
        offered = candidates(self.blk)
        self.assertEqual(sorted(offered["throttle_up"]), [59, 73, 201])

    def test_modifier_combos_are_skipped(self):
        # A block with two keys is a combo the autopilot cannot press.
        blk = {"throttle_rangeMax": [{"keyboardKey": ["56", "29"]}]}
        self.assertEqual(resolve_bindings(blk)["throttle_up"], [73])  # fallback

    def test_lone_extended_binding_is_used(self):
        blk = {"elevator_rangeMax": [{"keyboardKey": ["201"]}]}
        self.assertEqual(resolve_bindings(blk)["elevator_up"], [201])

    def test_overrides_win(self):
        bindings = resolve_bindings(self.blk, {"aileron_left": ["NUMPAD4"]})
        self.assertEqual(bindings["aileron_left"], [75])

    def test_describe_is_readable(self):
        text = describe(resolve_bindings(self.blk))
        self.assertEqual(text["elevator_up"], "S")
        self.assertEqual(text["throttle_up"], "NUMPAD9")


class NavMathTest(unittest.TestCase):
    def test_wrap180(self):
        self.assertAlmostEqual(wrap180(0), 0.0)
        self.assertAlmostEqual(wrap180(190), -170.0)
        self.assertAlmostEqual(wrap180(-190), 170.0)
        self.assertAlmostEqual(wrap180(360), 0.0)

    def test_target_directly_ahead_is_no_error(self):
        error, distance = turn_to_target((0.5, 0.5), (1.0, 0.0), (0.9, 0.5))
        self.assertAlmostEqual(error, 0.0, places=6)
        self.assertAlmostEqual(distance, 0.4, places=6)

    def test_sign_flips_the_turn_direction(self):
        kwargs = dict(own_xy=(0.5, 0.5), own_facing=(1.0, 0.0), target_xy=(0.5, 0.9))
        plus, _ = turn_to_target(sign=1, **kwargs)
        minus, _ = turn_to_target(sign=-1, **kwargs)
        self.assertAlmostEqual(plus, -minus, places=6)

    def test_bearing_uses_the_compass_frame(self):
        # Facing map +x, compass reads 90.  Target 90 deg clockwise from facing
        # in map space is therefore compass 180 when the map is not mirrored.
        bearing, error, _ = relative_bearing(
            (0.5, 0.5), (1.0, 0.0), (0.5, 0.9), compass_deg=90.0, sign=1
        )
        self.assertAlmostEqual(error, 90.0, places=4)
        self.assertAlmostEqual(bearing, 180.0, places=4)

    def test_sign_calibrator_learns_inverted_map(self):
        cal = SignCalibrator()
        # Map angle runs opposite to compass heading, as in the reference capture.
        for compass in range(0, 180, 10):
            angle = math.radians(217.5 - compass)
            cal.observe((math.cos(angle), math.sin(angle)), compass)
        self.assertGreaterEqual(cal.votes, SignCalibrator.MIN_VOTES)
        self.assertEqual(cal.sign(default=1), -1)

    def test_sign_calibrator_defaults_before_evidence(self):
        cal = SignCalibrator()
        self.assertEqual(cal.sign(default=1), 1)
        cal.observe((1.0, 0.0), 0.0)
        self.assertEqual(cal.sign(default=1), 1)


class AxisShapingTest(unittest.TestCase):
    def test_axis_state_deadband(self):
        self.assertEqual(axis_state(0.0, 0.1), 0)
        self.assertEqual(axis_state(0.05, 0.1), 0)
        self.assertEqual(axis_state(0.5, 0.1), 1)
        self.assertEqual(axis_state(-0.5, 0.1), -1)

    def test_dither_duty_tracks_demand(self):
        axis = DitherAxis(deadband=0.0)
        # A demand of 0.25 should press roughly a quarter of the ticks.
        presses = sum(1 for _ in range(400) if axis.step(0.25))
        self.assertGreater(presses, 70)
        self.assertLess(presses, 130)

    def test_dither_full_demand_is_continuous(self):
        axis = DitherAxis(deadband=0.0)
        self.assertTrue(all(axis.step(1.0) == 1 for _ in range(50)))

    def test_dither_releases_inside_deadband(self):
        axis = DitherAxis(deadband=0.1)
        self.assertEqual(axis.step(0.02), 0)


class ControllerTest(unittest.TestCase):
    """Closed-loop tests against the simulator."""

    def _run(self, mode, seconds, config=None, dt=0.05, altitude=None,
             sim=None) -> tuple[Autopilot, FlightSim]:
        config = config or Config()
        config.settings.source = "sim"
        config.settings.sim_time_scale = 1.0
        if altitude is not None:
            config.params.target_altitude_m = altitude
        set_calibration(config, roll=-1, pitch=-1, map_sign=-1)

        sim = sim if sim is not None else FlightSim()
        ap = Autopilot(sim, config, sink=NullSink())
        ap.time_scale = 1.0
        self.assertTrue(ap.engage(mode))
        # The tests drive tick() themselves; kill the timer thread engage() spun
        # up so the run is deterministic.
        ap.stop()
        for _ in range(int(seconds / dt)):
            ap.tick(dt)
        return ap, sim

    def test_stabilize_levels_the_aircraft(self):
        start = FlightSim()
        start.roll, start.pitch = 35.0, -12.0
        ap, sim = self._run("stabilize", 40.0, sim=start)
        self.assertLess(abs(sim.roll), 4.0, "wings should come level")
        self.assertLess(abs(sim.pitch), 3.0, "pitch should settle near zero")

    def test_altitude_hold_reaches_target(self):
        # 90 s leaves the two-segment law ~50 m short of 1600 m; the old
        # all-PD law needed a 120 m tolerance for the same run.
        ap, sim = self._run("alt_hold", 90.0, altitude=1600.0)
        self.assertLess(abs(sim.altitude - 1600.0), 60.0)

    def test_altitude_hold_descends_when_target_is_lower(self):
        ap, sim = self._run("alt_hold", 90.0, altitude=400.0)
        self.assertLess(abs(sim.altitude - 400.0), 60.0)

    def test_navigate_flies_over_the_point_of_interest(self):
        # The sim's turn rate is physical (~3 deg/s at 45 deg of bank for
        # 190 m/s), so a 120 deg heading change takes about 40 s.  Give the run
        # enough time to actually reach a target that starts ~79 km away.
        config = Config()
        config.settings.source = "sim"
        set_calibration(config, roll=-1, pitch=-1, map_sign=-1)
        sim = FlightSim()
        sim.altitude = 1200.0
        ap = Autopilot(sim, config, sink=NullSink())
        ap.time_scale = 1.0
        ap.engage("navigate")

        start = sim.poi_distance_m()
        closest = start
        best_error = 999.0
        arrived = False
        for _ in range(12000):  # 600 s of flight at 190 m/s
            ap.tick(0.05)
            closest = min(closest, sim.poi_distance_m())
            best_error = min(best_error, abs(sim.heading_error_to_poi_deg()))
            arrived = arrived or ap.command.arrived

        self.assertLess(closest, 1000.0, "should fly over the POI")
        self.assertGreater(closest, 0.0)
        self.assertLess(best_error, 5.0, "should point straight at the target")
        self.assertTrue(arrived, "the controller should report arrival")
        # Altitude is held throughout the turn.
        self.assertLess(abs(sim.altitude - 1200.0), 150.0)

    def test_navigate_ignores_the_configured_target_altitude(self):
        """target_altitude_m is an alt-hold-only setting: navigate holds the
        altitude it had when it was engaged instead of flying to that target."""
        config = Config()
        config.settings.source = "sim"
        config.params.target_altitude_m = 3500.0   # must not be used here
        set_calibration(config, roll=-1, pitch=-1, map_sign=-1)
        sim = FlightSim()
        sim.altitude = 900.0
        ap = Autopilot(sim, config, sink=NullSink())
        ap.time_scale = 1.0
        ap.engage("navigate")
        for _ in range(2400):  # 120 s
            ap.tick(0.05)
        self.assertLess(abs(sim.altitude - 900.0), 150.0)

    def test_navigate_turns_toward_the_target(self):
        config = Config()
        config.settings.source = "sim"
        set_calibration(config, roll=-1, pitch=-1, map_sign=-1)
        sim = FlightSim()
        ap = Autopilot(sim, config, sink=NullSink())
        ap.time_scale = 1.0
        ap.engage("navigate")

        initial_error = abs(sim.heading_error_to_poi_deg())
        self.assertGreater(initial_error, 90.0, "start pointing away from the POI")
        for _ in range(2400):  # 120 s
            ap.tick(0.05)
        self.assertLess(abs(sim.heading_error_to_poi_deg()), initial_error * 0.5)

    def test_map_sign_calibrates_from_motion(self):
        config = Config()
        config.settings.source = "sim"
        set_calibration(config, roll=-1, pitch=-1)
        config.calibration.map_sign = 1  # deliberately wrong
        sim = FlightSim()
        ap = Autopilot(sim, config, sink=NullSink())
        ap.time_scale = 1.0
        ap.engage("navigate")
        for _ in range(1200):  # 60 s
            ap.tick(0.05)
        self.assertEqual(config.calibration.map_sign, -1)

    def test_safety_disengages_below_minimum_altitude(self):
        config = Config()
        config.settings.source = "sim"
        config.params.min_altitude_m = 800.0
        set_calibration(config, roll=-1, pitch=-1, map_sign=-1)
        sim = FlightSim()
        sim.altitude = 700.0  # already below the floor
        ap = Autopilot(sim, config, sink=NullSink())
        ap.time_scale = 1.0
        ap.engage("navigate")
        for _ in range(200):
            ap.tick(0.05)
            if not ap.status()["engaged"]:
                break
        self.assertFalse(ap.status()["engaged"])
        self.assertIn("安全", ap.status()["message"])

    def test_blank_targets_need_explicit_flag_to_clear(self):
        """A blank UI field posts null: ignored for targets by default, clears with flag."""
        config = Config()
        config.params.target_heading_deg = 55.0
        config.params.target_altitude_m = 1200.0
        ap = Autopilot(FlightSim(), config, sink=NullSink())
        # Without the flag, null targets are ignored
        ap.update_params({"target_heading_deg": None, "target_altitude_m": None,
                          "tick_hz": None, "max_bank_deg": None})
        self.assertEqual(config.params.target_heading_deg, 55.0)  # preserved
        self.assertEqual(config.params.target_altitude_m, 1200.0)  # preserved
        self.assertEqual(config.params.tick_hz, 20.0)
        self.assertEqual(config.params.max_bank_deg, 45.0)
        # With the flag, null targets are accepted
        ap.update_params({"target_heading_deg": None, "target_altitude_m": None},
                         allow_null_targets=True)
        self.assertIsNone(config.params.target_heading_deg)
        self.assertIsNone(config.params.target_altitude_m)

    # ---- the two-segment altitude law ---------------------------------------

    def _bare_controller(self) -> FlightController:
        config = Config()
        ctl = FlightController(config.params)
        ctl.calibration = config.calibration
        return ctl

    def test_far_band_holds_a_constant_pitch(self):
        """Requirement 1: past the band the target is a fixed attitude, not a
        proportional response, and the climb rate does not enter the law."""
        for error, vario in ((250.0, 0.0), (5000.0, 25.0), (9000.0, -40.0),
                             (-250.0, 0.0), (-5000.0, -25.0), (-9000.0, 40.0)):
            ctl = self._bare_controller()
            cmd = Command()
            ctl._compute_pitch(cmd, 1000.0, vario, 0.05, 1000.0 + error)
            self.assertAlmostEqual(cmd.pitch_target,
                                   math.copysign(ALT_FAR_PITCH_DEG, error),
                                   places=6)
            self.assertTrue(cmd.altitude_zone.startswith("far"))

    def test_far_band_attitude_is_actually_held(self):
        """Requirement 1 in closed loop: mid-climb the aircraft sits at a steady
        pitch instead of chasing the (still enormous) altitude error."""
        ap, sim = self._run("alt_hold", 25.0, altitude=1600.0)
        self.assertGreater(1600.0 - sim.altitude, ALT_FAR_BAND_M)
        self.assertAlmostEqual(ap.command.pitch_target, ALT_FAR_PITCH_DEG, delta=0.3)

        samples = []
        for _ in range(200):  # 10 s
            ap.tick(0.05)
            samples.append(sim.pitch)
        self.assertLess(max(samples) - min(samples), 1.0, "pitch should be held flat")
        self.assertGreater(min(samples), 3.0, "and held nose-up, not sagging to level")

    def test_error_crosses_into_the_pd_band(self):
        """Requirement 2 (part 1): the loop does close on the target."""
        ap, sim = self._run("alt_hold", 90.0, altitude=1600.0)
        self.assertLess(abs(1600.0 - sim.altitude), ALT_FAR_BAND_M)

    def test_pd_takes_over_inside_the_band(self):
        """Requirement 2 (part 2): inside the band the target is the PD law."""
        config = Config()
        config.params.target_altitude_m = 1600.0
        set_calibration(config, roll=-1, pitch=-1, map_sign=-1)

        frame = FlightSim().read()
        frame.state["H, m"] = 1500.0        # 100 m below target -> inside the band
        frame.indicators["vario"] = 4.0
        ctl = FlightController(config.params)
        ctl.calibration = config.calibration
        cmd = ctl.update(frame, Mode.ALT_HOLD, 0.05)

        expected = config.params.alt_kp * 100.0 - config.params.vario_kd * 4.0
        self.assertAlmostEqual(cmd.pitch_target, expected, places=6)
        self.assertTrue(cmd.altitude_zone.startswith("pd"))

    def test_pitch_target_is_slew_limited(self):
        """The far/near hand-off must ramp, not step."""
        ctl = self._bare_controller()
        dt = 0.05
        max_step = PITCH_SLEW_DEG_PER_S * dt

        cmd = Command()
        ctl._compute_pitch(cmd, 1000.0, 0.0, dt, 1500.0)       # far band -> +7
        self.assertAlmostEqual(cmd.pitch_target, ALT_FAR_PITCH_DEG, places=6)

        previous = cmd.pitch_target
        steps = []
        for _ in range(60):
            cmd = Command()
            ctl._compute_pitch(cmd, 1440.0, 0.0, dt, 1000.0)   # far band -> -7
            steps.append(cmd.pitch_target - previous)
            previous = cmd.pitch_target

        self.assertTrue(all(abs(s) <= max_step + 1e-9 for s in steps))
        self.assertEqual(len([s for s in steps if s < 0]), 28,
                         "14 deg at 10 deg/s should take exactly 28 ticks")
        self.assertAlmostEqual(cmd.pitch_target, -ALT_FAR_PITCH_DEG, places=6)

    def test_pitch_law_leaves_roll_and_rudder_alone(self):
        """Requirement 3: the altitude channel must not leak into roll or yaw."""
        config = Config()
        config.params.target_heading_deg = 65.0
        config.params.use_rudder = True
        set_calibration(config, roll=-1, pitch=-1, map_sign=-1)

        def demand(altitude):
            frame = FlightSim().read()
            frame.state["H, m"] = altitude
            ctl = FlightController(config.params)
            ctl.calibration = config.calibration
            return ctl.update(frame, Mode.ALT_HOLD, 0.05)

        below = demand(0.0)      # 1000 m below target -> far band
        above = demand(1500.0)   # 500 m above target -> far band
        self.assertGreater(abs(below.rudder), 0.0)
        self.assertAlmostEqual(below.roll_target, above.roll_target, places=6)
        self.assertAlmostEqual(below.rudder, above.rudder, places=6)
        self.assertNotAlmostEqual(below.pitch_target, above.pitch_target, places=3)

        config.params.control_strategy = "pd"
        pd_bank = demand(0.0).roll_target
        config.params.control_strategy = "segmented"
        seg_bank = demand(0.0).roll_target
        self.assertNotAlmostEqual(pd_bank, seg_bank, places=3,
                                  msg="the strategy still governs the heading channel")

    def test_far_band_demand_clears_the_deadband(self):
        """A 7 deg attitude must move the elevator key, or the band is a no-op."""
        config = Config()
        ap, sim = self._run("alt_hold", 20.0, config=config, altitude=3000.0)
        self.assertGreater(3000.0 - sim.altitude, ALT_FAR_BAND_M)
        self.assertGreater(abs(ap.command.elevator), config.params.axis_deadband)


class KeyOutputTest(unittest.TestCase):
    def _static_source(self, frame):
        class Source:
            def read(self_inner):
                return frame

            def apply(self_inner, command, dt):
                return None

            def close(self_inner):
                return None

        return Source()

    def test_positive_aileron_presses_the_right_key(self):
        config = Config()
        config.settings.source = "http"      # keep 'injecting' semantics simple
        config.settings.require_focus = False
        set_calibration(config, roll=1, pitch=1, map_sign=1)

        sim = FlightSim()
        frame = sim.read()
        frame.indicators["aviahorizon_roll"] = 25.0   # banked right
        frame.indicators["aviahorizon_pitch"] = 0.0
        sink = NullSink()
        ap = Autopilot(self._static_source(frame), config, sink=sink)
        ap.time_scale = 1.0

        command = ap.controller.update(frame, Mode.STABILIZE, 0.05)
        self.assertLess(command.aileron, 0, "must roll left to level the wings")
        ap._apply_axes(command)
        for _ in range(20):
            ap._apply_axes(command)
        pressed = {code for _, kind, code in sink.log if kind == "down"}
        self.assertEqual(pressed, {30}, "only A (roll left) should be held, not D")

    def test_elevator_key_matches_nose_up_demand(self):
        config = Config()
        config.settings.source = "http"
        config.settings.require_focus = False
        set_calibration(config, roll=1, pitch=1, map_sign=1)
        sim = FlightSim()
        frame = sim.read()
        config.params.target_altitude_m = frame.altitude_m + 500.0
        sink = NullSink()
        ap = Autopilot(self._static_source(frame), config, sink=sink)
        ap.time_scale = 1.0
        ap.controller.capture_hold_targets(frame)

        command = ap.controller.update(frame, Mode.ALT_HOLD, 0.05)
        self.assertGreater(command.pitch_target, 0, "climb demand should pitch up")
        for _ in range(20):
            ap._apply_axes(command)
        pressed = {code for _, kind, code in sink.log if kind == "down"}
        self.assertIn(31, pressed, "S pitches the nose up")


class FixtureSourceTest(unittest.TestCase):
    def test_reads_the_recorded_capture(self):
        frame = FixtureSource(REF_DIR).read()
        self.assertTrue(frame.valid)
        self.assertIsNotNone(frame.altitude_m)
        self.assertIsNotNone(frame.heading_deg)
        self.assertIsNotNone(frame.player)
        self.assertIsNotNone(frame.point_of_interest)
        self.assertAlmostEqual(frame.heading_deg, 187.872437, places=3)

    def test_summary_is_json_serialisable(self):
        import json

        frame = FixtureSource(REF_DIR).read()
        json.dumps(frame.summary())

    def test_poi_distance_is_plausible_without_map_info(self):
        frame = FixtureSource(REF_DIR).read()
        player = frame.player
        poi = frame.point_of_interest
        distance = math.hypot(poi.x - player.x, poi.y - player.y) * frame.meters_per_map_unit
        self.assertGreater(distance, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)