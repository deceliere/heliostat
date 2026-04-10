import json
import os
import signal
import sys
import time

import paho.mqtt.client as mqtt


MQTT_HOST = os.getenv("HELIOSTAT_MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("HELIOSTAT_MQTT_PORT", "1883"))
MQTT_BASE_TOPIC = os.getenv("HELIOSTAT_MQTT_BASE_TOPIC", "heliostat/remote1")
STATE_PERIOD_S = 0.5


runtime = {
    "mode": "manual",
    "pan_deg": 90.0,
    "tilt_deg": 90.0,
    "pan_target_deg": 90.0,
    "tilt_target_deg": 90.0,
    "pan_us": 1491,
    "tilt_us": 1500,
    "sun_time_ok": True,
    "target_ok": False,
    "remote_utc": int(time.time()),
    "auto_enabled": False,
    "wifi_ok": True,
    "mqtt_ok": True,
    "ntp_ok": True,
    "time_source": "ntp",
}

running = True


def topic(suffix: str) -> str:
    return f"{MQTT_BASE_TOPIC}/{suffix}"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def angle_to_us(angle_deg: float) -> int:
    return round(500 + ((angle_deg / 180.0) * 2000.0))


def publish_state(client: mqtt.Client) -> None:
    runtime["pan_us"] = angle_to_us(runtime["pan_deg"])
    runtime["tilt_us"] = angle_to_us(runtime["tilt_deg"])
    runtime["remote_utc"] = int(time.time())
    client.publish(topic("state"), json.dumps(runtime), qos=0, retain=True)


def publish_diag(client: mqtt.Client, event: str, extra: dict | None = None) -> None:
    payload = {
        "event": event,
        "mode": runtime["mode"],
        "pan_deg": runtime["pan_deg"],
        "tilt_deg": runtime["tilt_deg"],
        "remote_utc": int(time.time()),
    }
    if extra:
        payload.update(extra)
    client.publish(topic("diag"), json.dumps(payload), qos=0, retain=False)


def handle_mode(client: mqtt.Client, payload: dict) -> None:
    mode = payload.get("mode")
    if mode not in {"manual", "captured", "auto"}:
        publish_diag(client, "invalid_mode", {"payload": payload})
        return
    runtime["mode"] = mode
    runtime["auto_enabled"] = mode == "auto"
    publish_diag(client, "mode_changed", {"new_mode": mode})


def handle_jog(client: mqtt.Client, payload: dict) -> None:
    axis = payload.get("axis")
    delta_deg = float(payload.get("delta_deg", 0.0))
    if axis == "pan":
        runtime["pan_target_deg"] = clamp(runtime["pan_target_deg"] + delta_deg, 0.0, 180.0)
    elif axis == "tilt":
        runtime["tilt_target_deg"] = clamp(runtime["tilt_target_deg"] + delta_deg, 0.0, 180.0)
    else:
        publish_diag(client, "invalid_jog", {"payload": payload})
        return

    runtime["mode"] = "manual"
    runtime["auto_enabled"] = False


def handle_move_to(client: mqtt.Client, payload: dict) -> None:
    runtime["pan_target_deg"] = clamp(float(payload.get("pan_deg", runtime["pan_target_deg"])), 0.0, 180.0)
    runtime["tilt_target_deg"] = clamp(float(payload.get("tilt_deg", runtime["tilt_target_deg"])), 0.0, 180.0)
    runtime["mode"] = "manual"
    runtime["auto_enabled"] = False


def handle_action(client: mqtt.Client, payload: dict) -> None:
    action = payload.get("action")
    if action == "capture_target":
        runtime["target_ok"] = True
        runtime["mode"] = "captured"
        runtime["auto_enabled"] = False
        publish_diag(client, "capture_target")
    elif action == "recenter":
        runtime["pan_target_deg"] = 90.0
        runtime["tilt_target_deg"] = 90.0
        runtime["mode"] = "manual"
        runtime["auto_enabled"] = False
        publish_diag(client, "recenter")
    elif action == "print_diag":
        publish_diag(client, "print_diag")
    else:
        publish_diag(client, "invalid_action", {"payload": payload})


def handle_time(client: mqtt.Client, payload: dict) -> None:
    runtime["remote_utc"] = int(payload.get("unix_utc", int(time.time())))
    publish_diag(client, "time_set", {"remote_utc": runtime["remote_utc"]})


def on_connect(client: mqtt.Client, _userdata, _flags, reason_code, _properties=None) -> None:
    client.subscribe(topic("cmd/mode"))
    client.subscribe(topic("cmd/jog"))
    client.subscribe(topic("cmd/move_to"))
    client.subscribe(topic("cmd/action"))
    client.subscribe(topic("cmd/time"))
    client.publish(topic("availability"), "online", qos=1, retain=True)
    publish_state(client)
    publish_diag(client, "simulator_started")


def on_message(client: mqtt.Client, _userdata, message: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(message.payload.decode("utf-8"))
    except json.JSONDecodeError:
        publish_diag(client, "invalid_json", {"topic": message.topic})
        return

    if message.topic == topic("cmd/mode"):
        handle_mode(client, payload)
    elif message.topic == topic("cmd/jog"):
        handle_jog(client, payload)
    elif message.topic == topic("cmd/move_to"):
        handle_move_to(client, payload)
    elif message.topic == topic("cmd/action"):
        handle_action(client, payload)
    elif message.topic == topic("cmd/time"):
        handle_time(client, payload)
    publish_state(client)


def stop_handler(_signum, _frame) -> None:
    global running
    running = False


def tick_state() -> None:
    if runtime["mode"] == "auto":
        runtime["pan_target_deg"] = clamp(runtime["pan_target_deg"] + 0.03, 0.0, 180.0)
        runtime["tilt_target_deg"] = clamp(runtime["tilt_target_deg"] + 0.01, 0.0, 180.0)

    max_step = 15.0 * STATE_PERIOD_S
    pan_delta = runtime["pan_target_deg"] - runtime["pan_deg"]
    tilt_delta = runtime["tilt_target_deg"] - runtime["tilt_deg"]
    runtime["pan_deg"] += clamp(pan_delta, -max_step, max_step)
    runtime["tilt_deg"] += clamp(tilt_delta, -max_step, max_step)


def main() -> int:
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="heliostat-simulated-remote")
    client.will_set(topic("availability"), "offline", qos=1, retain=True)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
    client.loop_start()

    try:
        while running:
            tick_state()
            publish_state(client)
            time.sleep(STATE_PERIOD_S)
    finally:
        client.publish(topic("availability"), "offline", qos=1, retain=True)
        client.loop_stop()
        client.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
