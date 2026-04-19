import json
import os
import threading
import time
from pathlib import Path

from fastapi import FastAPI
from fastapi import HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import paho.mqtt.client as mqtt


BASE_DIR = Path(__file__).resolve().parent.parent
STATIC_DIR = BASE_DIR / "static"
DATA_DIR = BASE_DIR / "data"
PRESETS_PATH = DATA_DIR / "presets.json"
MQTT_HOST = os.getenv("HELIOSTAT_MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("HELIOSTAT_MQTT_PORT", "1883"))
MQTT_BASE_TOPIC = os.getenv("HELIOSTAT_MQTT_BASE_TOPIC", "heliostat/remote1")

app = FastAPI(title="Heliostat Remote Access")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


state_lock = threading.Lock()
runtime_state = {
    "mqtt_connected": False,
    "remote_online": False,
    "mode": "unknown",
    "pan_deg": None,
    "tilt_deg": None,
    "pan_target_deg": None,
    "tilt_target_deg": None,
    "pan_us": None,
    "tilt_us": None,
    "site_latitude_deg": None,
    "site_longitude_deg": None,
    "scan_active": False,
    "scan_paused": False,
    "scan_stage": "idle",
    "scan_direction": "forward",
    "scan_center_pan_deg": None,
    "scan_center_tilt_deg": None,
    "scan_range_pan_deg": None,
    "scan_range_tilt_deg": None,
    "scan_step_deg": None,
    "scan_dwell_ms": None,
    "scan_move_speed_deg_per_sec": None,
    "scan_point_index": 0,
    "scan_points_total": 0,
    "scan_lock_valid": False,
    "scan_lock_pan_deg": None,
    "scan_lock_tilt_deg": None,
    "approx_target_valid": False,
    "approx_target_bearing_deg": None,
    "approx_target_elevation_deg": None,
    "approx_target_pan_deg": None,
    "approx_target_tilt_deg": None,
    "remote_utc": None,
    "sun_time_ok": False,
    "target_ok": False,
    "ntp_ok": False,
    "time_source": "unknown",
    "last_state_update_unix_ms": 0,
    "last_diag": None,
}

PRESET_GROUPS = {"site_locations", "beam_directions"}


def set_state(**kwargs) -> None:
    with state_lock:
        runtime_state.update(kwargs)


def get_state() -> dict:
    with state_lock:
        return dict(runtime_state)


def topic(suffix: str) -> str:
    return f"{MQTT_BASE_TOPIC}/{suffix}"


def default_presets() -> dict:
    return {
        "site_locations": [],
        "beam_directions": [],
    }


def load_presets() -> dict:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    if not PRESETS_PATH.exists():
        presets = default_presets()
        PRESETS_PATH.write_text(json.dumps(presets, indent=2) + "\n", encoding="utf-8")
        return presets

    try:
        loaded = json.loads(PRESETS_PATH.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        loaded = {}

    presets = default_presets()
    if isinstance(loaded, dict):
        for key in PRESET_GROUPS:
            values = loaded.get(key, [])
            if isinstance(values, list):
                presets[key] = values
    return presets


def save_presets(presets: dict) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PRESETS_PATH.write_text(json.dumps(presets, indent=2) + "\n", encoding="utf-8")


def normalize_name(name: str) -> str:
    normalized = str(name).strip()
    if not normalized:
        raise HTTPException(status_code=400, detail="name is required")
    return normalized


def upsert_preset(group: str, preset: dict) -> dict:
    if group not in PRESET_GROUPS:
        raise HTTPException(status_code=400, detail="invalid preset group")
    presets = load_presets()
    items = [item for item in presets[group] if item.get("name") != preset["name"]]
    items.append(preset)
    items.sort(key=lambda item: item.get("name", "").lower())
    presets[group] = items
    save_presets(presets)
    return presets


def delete_preset(group: str, name: str) -> dict:
    if group not in PRESET_GROUPS:
        raise HTTPException(status_code=400, detail="invalid preset group")
    presets = load_presets()
    presets[group] = [item for item in presets[group] if item.get("name") != name]
    save_presets(presets)
    return presets


def mqtt_reason_code_is_success(reason_code) -> bool:
    is_failure = getattr(reason_code, "is_failure", None)
    if is_failure is not None:
        return not is_failure

    value = getattr(reason_code, "value", reason_code)
    try:
        return int(value) == 0
    except (TypeError, ValueError):
        return str(reason_code).lower() == "success"


def on_connect(client: mqtt.Client, _userdata, _flags, reason_code, _properties=None) -> None:
    connected = mqtt_reason_code_is_success(reason_code)
    set_state(mqtt_connected=connected)
    if not connected:
        return
    client.subscribe(topic("state"))
    client.subscribe(topic("diag"))
    client.subscribe(topic("availability"))


def on_disconnect(_client: mqtt.Client, _userdata, _disconnect_flags, _reason_code, _properties=None) -> None:
    set_state(mqtt_connected=False)


def on_message(_client: mqtt.Client, _userdata, message: mqtt.MQTTMessage) -> None:
    payload = message.payload.decode("utf-8", errors="replace")

    if message.topic == topic("availability"):
      set_state(remote_online=(payload.strip().lower() == "online"))
      return

    if message.topic == topic("state"):
      try:
        data = json.loads(payload)
      except json.JSONDecodeError:
        return
      set_state(
          mode=data.get("mode", "unknown"),
          pan_deg=data.get("pan_deg"),
          tilt_deg=data.get("tilt_deg"),
          pan_target_deg=data.get("pan_target_deg"),
          tilt_target_deg=data.get("tilt_target_deg"),
          pan_us=data.get("pan_us"),
          tilt_us=data.get("tilt_us"),
          site_latitude_deg=data.get("site_latitude_deg"),
          site_longitude_deg=data.get("site_longitude_deg"),
          scan_active=bool(data.get("scan_active", False)),
          scan_paused=bool(data.get("scan_paused", False)),
          scan_stage=data.get("scan_stage", "idle"),
          scan_direction=data.get("scan_direction", "forward"),
          scan_center_pan_deg=data.get("scan_center_pan_deg"),
          scan_center_tilt_deg=data.get("scan_center_tilt_deg"),
          scan_range_pan_deg=data.get("scan_range_pan_deg"),
          scan_range_tilt_deg=data.get("scan_range_tilt_deg"),
          scan_step_deg=data.get("scan_step_deg"),
          scan_dwell_ms=data.get("scan_dwell_ms"),
          scan_move_speed_deg_per_sec=data.get("scan_move_speed_deg_per_sec"),
          scan_point_index=data.get("scan_point_index", 0),
          scan_points_total=data.get("scan_points_total", 0),
          scan_lock_valid=bool(data.get("scan_lock_valid", False)),
          scan_lock_pan_deg=data.get("scan_lock_pan_deg"),
          scan_lock_tilt_deg=data.get("scan_lock_tilt_deg"),
          approx_target_valid=bool(data.get("approx_target_valid", False)),
          approx_target_bearing_deg=data.get("approx_target_bearing_deg"),
          approx_target_elevation_deg=data.get("approx_target_elevation_deg"),
          approx_target_pan_deg=data.get("approx_target_pan_deg"),
          approx_target_tilt_deg=data.get("approx_target_tilt_deg"),
          remote_utc=data.get("remote_utc"),
          sun_time_ok=bool(data.get("sun_time_ok", False)),
          target_ok=bool(data.get("target_ok", False)),
          ntp_ok=bool(data.get("ntp_ok", False)),
          time_source=data.get("time_source", "unknown"),
          last_state_update_unix_ms=int(round(time.time() * 1000)),
          remote_online=bool(data.get("mqtt_ok", get_state().get("remote_online", False))),
      )
      return

    if message.topic == topic("diag"):
      try:
        data = json.loads(payload)
      except json.JSONDecodeError:
        data = {"raw": payload}
      set_state(last_diag=data)


mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="heliostat-ui")
mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message = on_message


@app.on_event("startup")
def startup_event() -> None:
    mqtt_client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
    mqtt_client.loop_start()


@app.on_event("shutdown")
def shutdown_event() -> None:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()


@app.get("/api/health")
def api_health() -> dict:
    state = get_state()
    return {
        "ok": True,
        "service": "heliostat-ui",
        "mqtt_connected": state["mqtt_connected"],
        "remote_online": state["remote_online"],
        "note": "MQTT bridge active",
    }


@app.get("/api/state")
def api_state() -> dict:
    return get_state()


@app.get("/api/presets")
def api_presets() -> dict:
    return load_presets()


@app.post("/api/presets/site")
def api_preset_site(payload: dict) -> dict:
    preset = {
        "name": normalize_name(payload.get("name", "")),
        "latitude_deg": float(payload.get("latitude_deg")),
        "longitude_deg": float(payload.get("longitude_deg")),
    }
    presets = upsert_preset("site_locations", preset)
    return {"ok": True, "presets": presets, "preset": preset}


@app.post("/api/presets/site/delete")
def api_preset_site_delete(payload: dict) -> dict:
    presets = delete_preset("site_locations", normalize_name(payload.get("name", "")))
    return {"ok": True, "presets": presets}


@app.post("/api/presets/beam")
def api_preset_beam(payload: dict) -> dict:
    preset = {
        "name": normalize_name(payload.get("name", "")),
        "bearing_deg": float(payload.get("bearing_deg")),
        "elevation_deg": float(payload.get("elevation_deg")),
    }
    presets = upsert_preset("beam_directions", preset)
    return {"ok": True, "presets": presets, "preset": preset}


@app.post("/api/presets/beam/delete")
def api_preset_beam_delete(payload: dict) -> dict:
    presets = delete_preset("beam_directions", normalize_name(payload.get("name", "")))
    return {"ok": True, "presets": presets}


@app.post("/api/cmd/mode")
def api_cmd_mode(payload: dict) -> dict:
    mode = payload.get("mode")
    if mode not in {"manual", "captured", "auto"}:
        raise HTTPException(status_code=400, detail="invalid mode")
    mqtt_client.publish(topic("cmd/mode"), json.dumps({"mode": mode}), qos=1)
    return {"ok": True, "published": True}


@app.post("/api/cmd/action")
def api_cmd_action(payload: dict) -> dict:
    action = payload.get("action")
    if action not in {"capture_target", "recenter", "print_diag", "beam_seen"}:
        raise HTTPException(status_code=400, detail="invalid action")
    mqtt_client.publish(topic("cmd/action"), json.dumps({"action": action}), qos=1)
    return {"ok": True, "published": True}


@app.post("/api/cmd/jog")
def api_cmd_jog(payload: dict) -> dict:
    axis = payload.get("axis")
    if axis not in {"pan", "tilt"}:
        raise HTTPException(status_code=400, detail="invalid axis")

    delta_deg = float(payload.get("delta_deg", 0.0))
    if delta_deg == 0.0:
        raise HTTPException(status_code=400, detail="delta_deg must be non-zero")

    command = {"axis": axis, "delta_deg": delta_deg}
    mqtt_client.publish(topic("cmd/jog"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/move-to")
def api_cmd_move_to(payload: dict) -> dict:
    if "pan_deg" not in payload or "tilt_deg" not in payload:
        raise HTTPException(status_code=400, detail="pan_deg and tilt_deg are required")

    command = {
        "pan_deg": float(payload.get("pan_deg")),
        "tilt_deg": float(payload.get("tilt_deg")),
    }
    mqtt_client.publish(topic("cmd/move_to"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/time")
def api_cmd_time(payload: dict) -> dict:
    unix_utc = int(payload.get("unix_utc", 0))
    if unix_utc <= 0:
        raise HTTPException(status_code=400, detail="invalid unix_utc")

    command = {
        "unix_utc": unix_utc,
        "time_scale": float(payload.get("time_scale", 1.0)),
    }
    mqtt_client.publish(topic("cmd/time"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/location")
def api_cmd_location(payload: dict) -> dict:
    command = {
        "latitude_deg": float(payload.get("latitude_deg")),
        "longitude_deg": float(payload.get("longitude_deg")),
    }
    mqtt_client.publish(topic("cmd/location"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/approx-target")
def api_cmd_approx_target(payload: dict) -> dict:
    command = {
        "bearing_deg": float(payload.get("bearing_deg")),
        "elevation_deg": float(payload.get("elevation_deg")),
    }
    mqtt_client.publish(topic("cmd/approx_target"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/scan/start")
def api_cmd_scan_start(payload: dict) -> dict:
    stage = payload.get("stage")
    if stage not in {"coarse", "fine", "micro"}:
        raise HTTPException(status_code=400, detail="invalid stage")

    command = {
        "stage": stage,
        "center_pan_deg": float(payload.get("center_pan_deg")),
        "center_tilt_deg": float(payload.get("center_tilt_deg")),
        "dwell_ms": int(payload.get("dwell_ms", 0)),
        "move_speed_deg_per_sec": float(payload.get("move_speed_deg_per_sec", 0.0)),
        "direction": payload.get("direction", "forward"),
    }
    if payload.get("range_pan_deg") is not None:
        command["range_pan_deg"] = float(payload.get("range_pan_deg"))
    if payload.get("range_tilt_deg") is not None:
        command["range_tilt_deg"] = float(payload.get("range_tilt_deg"))
    mqtt_client.publish(topic("cmd/scan"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/scan/stop")
def api_cmd_scan_stop() -> dict:
    command = {"stage": "off"}
    mqtt_client.publish(topic("cmd/scan"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/scan/resume")
def api_cmd_scan_resume(payload: dict) -> dict:
    direction = payload.get("direction", "forward")
    if direction not in {"forward", "backward"}:
        raise HTTPException(status_code=400, detail="invalid direction")
    command = {"stage": "resume", "direction": direction}
    mqtt_client.publish(topic("cmd/scan"), json.dumps(command), qos=1)
    return {"ok": True, "published": True, "command": command}


@app.post("/api/cmd/manual")
def api_cmd_manual(payload: dict) -> dict:
    command = {
        "pan_rate": float(payload.get("pan_rate", 0.0)),
        "tilt_rate": float(payload.get("tilt_rate", 0.0)),
        "precision": bool(payload.get("precision", False)),
    }
    mqtt_client.publish(topic("cmd/manual"), json.dumps(command), qos=0)
    return {"ok": True, "published": True, "command": command}


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")
