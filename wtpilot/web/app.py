"""Flask app: telemetry dashboard, mode selection, tuning, .blk upload."""

from __future__ import annotations

import os
import shutil
import threading

from flask import Flask, jsonify, render_template, request

from ..autopilot import Autopilot
from ..config import CONFIG_PATH, UPLOADED_BLK, Config
from ..keymap import candidates, describe, load_blk, resolve_bindings
from ..sim import FlightSim
from ..telemetry import FixtureSource, HttpSource

MAX_BLK_BYTES = 512 * 1024

# Actions the web UI lets the user rebind by name.
BINDABLE = (
    "elevator_up", "elevator_down", "aileron_left", "aileron_right",
    "rudder_left", "rudder_right", "throttle_up", "throttle_down",
)
TUNABLE = (
    "tick_hz", "max_bank_deg", "max_pitch_deg", "roll_kp", "roll_kd",
    "pitch_kp", "pitch_kd", "heading_kp", "alt_kp", "vario_kd",
    "target_altitude_m", "target_heading_deg", "arrive_radius_m",
    "axis_deadband", "min_altitude_m", "use_throttle", "throttle_target",
    "target_speed_kmh", "use_rudder", "control_strategy",
)


def is_dry_run(config: Config) -> bool:
    """A non-live source must never inject keystrokes."""
    return config.settings.dry_run or config.settings.source != "http"


def build_source(config: Config, ref_dir: str):
    if config.settings.source == "sim":
        return FlightSim()
    if config.settings.source == "fixture":
        return FixtureSource(ref_dir)
    if config.settings.source == "powerbroker":
        from ..telemetry import PowerBrokerSource
        return PowerBrokerSource(config.settings.base_url, config.settings.timeout_s)
    return HttpSource(config.settings.base_url, config.settings.timeout_s)


def create_app(ref_dir: str, config: Config | None = None,
               config_path: str = CONFIG_PATH) -> Flask:
    app = Flask(__name__)
    app.config["MAX_CONTENT_LENGTH"] = MAX_BLK_BYTES

    if config is None:
        config = Config.load(config_path)
    source = build_source(config, ref_dir)
    # The simulator must never leak keystrokes into whatever window is focused.
    autopilot = Autopilot(source, config, dry_run=is_dry_run(config))

    if config.settings.autoconnect:
        autopilot.start()

    def save_config() -> None:
        config.save(config_path)

    @app.get("/")
    def index():
        from ..keymap import SCANCODE_NAMES

        return render_template(
            "index.html",
            key_names=sorted(SCANCODE_NAMES.values()),
            bindable=BINDABLE,
        )

    @app.get("/api/status")
    def status():
        return jsonify(autopilot.status())

    @app.get("/api/config")
    def get_config():
        data = config.to_dict()
        data["tunable"] = list(TUNABLE)
        data["config_path"] = config_path
        return jsonify(data)

    @app.post("/api/settings")
    def set_settings():
        payload = request.get_json(silent=True) or {}
        changed_source = False
        for key in ("base_url", "require_focus", "focus_title", "autoconnect",
                    "timeout_s", "sim_time_scale"):
            if key in payload:
                setattr(config.settings, key, payload[key])
        if "source" in payload and payload["source"] != config.settings.source:
            config.settings.source = str(payload["source"])
            changed_source = True
        save_config()
        if changed_source:
            autopilot.set_source(
                build_source(config, ref_dir), dry_run=is_dry_run(config)
            )
            autopilot.start()
        return jsonify({"ok": True, "settings": config.to_dict()["settings"]})

    @app.post("/api/params")
    def set_params():
        payload = request.get_json(silent=True) or {}
        values = {k: v for k, v in payload.items() if k in TUNABLE}
        # When applying params via the "应用参数" button, null targets mean "clear them"
        autopilot.update_params(values, allow_null_targets=True)
        save_config()
        return jsonify({"ok": True, "params": config.to_dict()["params"]})

    @app.post("/api/engage")
    def engage():
        payload = request.get_json(silent=True) or {}
        mode = payload.get("mode", "stabilize")
        values = {k: v for k, v in payload.items() if k in TUNABLE}
        if values:
            # A blank target-altitude box is meaningful here: it means "hold the
            # altitude we are at", which capture_hold_targets() resolves.
            autopilot.update_params(values, allow_null_targets=True)
            save_config()
        ok = autopilot.engage(mode)
        return jsonify({"ok": ok, "message": autopilot.status()["message"]})

    @app.post("/api/disengage")
    def disengage():
        autopilot.disengage()
        return jsonify({"ok": True})

    @app.post("/api/focus")
    def focus():
        from ..keys import focus_window

        ok = focus_window(config.settings.focus_title)
        return jsonify({"ok": ok})

    @app.post("/api/upload_blk")
    def upload_blk():
        upload = request.files.get("file")
        if upload is None or not upload.filename:
            return jsonify({"ok": False, "error": "没有选择文件"}), 400
        if not upload.filename.lower().endswith(".blk"):
            return jsonify({"ok": False, "error": "只接受 .blk 文件"}), 400

        os.makedirs(os.path.dirname(UPLOADED_BLK), exist_ok=True)
        tmp_path = UPLOADED_BLK + ".tmp"
        upload.save(tmp_path)

        parsed = load_blk(tmp_path)
        if not parsed:
            os.remove(tmp_path)
            return jsonify({"ok": False, "error": "文件里没有找到按键绑定"}), 400

        shutil.move(tmp_path, UPLOADED_BLK)
        config.settings.blk_path = UPLOADED_BLK
        config.key_overrides = {}
        save_config()
        bindings = autopilot.reload_bindings(UPLOADED_BLK)
        return jsonify(
            {
                "ok": True,
                "filename": upload.filename,
                "path": UPLOADED_BLK,
                "bindings": bindings,
                "offered": describe(candidates(load_blk(UPLOADED_BLK))),
                "blocks": len(parsed),
            }
        )

    @app.get("/api/bindings")
    def get_bindings():
        return jsonify(
            {
                "blk_path": config.settings.blk_path,
                "bindings": describe(autopilot.bindings),
                "offered": describe(
                    candidates(load_blk(config.settings.blk_path))
                ),
                "overrides": config.key_overrides,
            }
        )

    @app.post("/api/bindings")
    def set_bindings():
        payload = request.get_json(silent=True) or {}
        overrides = {}
        for action, names in (payload.get("overrides") or {}).items():
            if action not in BINDABLE:
                continue
            cleaned = [str(n).strip().upper() for n in names if str(n).strip()]
            if cleaned:
                overrides[action] = cleaned
        config.key_overrides = overrides
        resolved = resolve_bindings(load_blk(config.settings.blk_path), overrides)
        save_config()
        autopilot.bindings = resolved
        return jsonify({"ok": True, "bindings": describe(resolved)})

    return app


def run(ref_dir: str, config: Config | None = None, host: str = "127.0.0.1",
        port: int = 8112, config_path: str = CONFIG_PATH,
        open_browser: bool = False) -> None:
    app = create_app(ref_dir, config, config_path)
    if open_browser:
        import webbrowser

        threading.Timer(1.0, lambda: webbrowser.open(f"http://{host}:{port}/")).start()
    app.run(host=host, port=port, threaded=True, use_reloader=False)