import json
import os
import threading
from pathlib import Path

from fastapi import FastAPI
from fastapi import HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import paho.mqtt.client as mqtt


BASE_DIR = Path(__file__).resolve().parent.parent
STATIC_DIR = BASE_DIR / "static"
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
    "pan_us": None,
    "tilt_us": None,
    "remote_utc": None,
    "sun_time_ok": False,
    "target_ok": False,
    "last_diag": None,
}


def set_state(**kwargs) -> None:
    with state_lock:
        runtime_state.update(kwargs)


def get_state() -> dict:
    with state_lock:
        return dict(runtime_state)


def topic(suffix: str) -> str:
    return f"{MQTT_BASE_TOPIC}/{suffix}"


def on_connect(client: mqtt.Client, _userdata, _flags, reason_code, _properties=None) -> None:
    connected = int(reason_code) == 0
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
          pan_us=data.get("pan_us"),
          tilt_us=data.get("tilt_us"),
          remote_utc=data.get("remote_utc"),
          sun_time_ok=bool(data.get("sun_time_ok", False)),
          target_ok=bool(data.get("target_ok", False)),
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
    if action not in {"capture_target", "recenter", "print_diag"}:
        raise HTTPException(status_code=400, detail="invalid action")
    mqtt_client.publish(topic("cmd/action"), json.dumps({"action": action}), qos=1)
    return {"ok": True, "published": True}


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
