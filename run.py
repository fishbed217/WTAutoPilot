#!/usr/bin/env python3
"""Entry point for the War Thunder autopilot.

    python run.py                      # live game on localhost:8111 + web UI
    python run.py --source sim --open  # offline simulator demo, no game needed
    python run.py --console navigate --source sim   # headless, prints status

The autopilot never touches port 8111 unless the source is ``http``.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from wtpilot.autopilot import Autopilot
from wtpilot.config import CONFIG_PATH, REF_DIR, Config
from wtpilot.web.app import build_source, is_dry_run, run as run_web


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", choices=("http", "sim", "fixture"),
                        help="telemetry source (default: whatever is in the config)")
    parser.add_argument("--blk", help="path to a .blk control file to use")
    parser.add_argument("--config", default=CONFIG_PATH, help="config file path")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8112)
    parser.add_argument("--open", action="store_true", help="open the browser")
    parser.add_argument("--dry-run", action="store_true",
                        help="compute commands but never press keys")
    parser.add_argument("--sim-time-scale", type=float,
                        help="fast-forward factor for the simulator")
    parser.add_argument("--no-focus-check", action="store_true",
                        help="send keys even when the game window is not focused")
    parser.add_argument("--console", metavar="MODE",
                        help="run headless in the given mode (stabilize/alt_hold/navigate)")
    parser.add_argument("--altitude", type=float, help="target altitude in metres")
    parser.add_argument("--seconds", type=float, default=0,
                        help="console mode: stop after this many seconds")
    return parser.parse_args(argv)


def build_config(args: argparse.Namespace) -> Config:
    config = Config.load(args.config)
    if args.source:
        config.settings.source = args.source
    if args.blk:
        config.settings.blk_path = args.blk
    if args.sim_time_scale is not None:
        config.settings.sim_time_scale = args.sim_time_scale
    if args.dry_run:
        config.settings.dry_run = True
    if args.no_focus_check:
        config.settings.require_focus = False
    if args.altitude is not None:
        config.params.target_altitude_m = args.altitude
    return config


def run_console(config: Config, mode: str, seconds: float) -> int:
    source = build_source(config, REF_DIR)
    autopilot = Autopilot(source, config, dry_run=is_dry_run(config))

    stopping = False

    def _stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, _stop)

    if not autopilot.engage(mode):
        print(f"cannot engage: {autopilot.status()['message']}", file=sys.stderr)
        autopilot.close()
        return 2

    autopilot.start()
    start = time.time()
    try:
        while not stopping:
            status = autopilot.status()
            telemetry = status["telemetry"] or {}
            nav = status["nav"] or {}
            altitude = telemetry.get("altitude_m")
            print(
                f"[{time.time() - start:6.1f}s] {status['mode']:<9} "
                f"alt={altitude if altitude is None else round(altitude)}m "
                f"ias={telemetry.get('ias_kmh') and round(telemetry['ias_kmh'])}km/h "
                f"hdg={telemetry.get('heading_deg') and round(telemetry['heading_deg'])} "
                f"roll={telemetry.get('roll_raw_deg') and round(telemetry['roll_raw_deg'], 1)} "
                f"pitch={telemetry.get('pitch_raw_deg') and round(telemetry['pitch_raw_deg'], 1)} "
                f"dist={nav.get('distance_m') and round(nav['distance_m'] / 1000, 2)}km "
                f"| {status['message']}"
            )
            if seconds and time.time() - start >= seconds:
                break
            time.sleep(1.0)
    finally:
        autopilot.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = build_config(args)
    if args.console:
        return run_console(config, args.console, args.seconds)
    run_web(REF_DIR, config, host=args.host, port=args.port,
            config_path=args.config, open_browser=args.open)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())