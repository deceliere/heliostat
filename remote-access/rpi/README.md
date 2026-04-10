# Raspberry Pi Skeleton

This directory contains the first draft of the workshop-side stack for remote access.

## Intended Components

- `app/`: FastAPI app serving the UI and later bridging to MQTT
- `static/`: browser UI
- `systemd/`: service unit examples
- `requirements.txt`: Python dependencies

## First Goal

Get a lightweight local web app running on the Raspberry Pi without changing the ESP32 yet.

## Suggested First Run

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8088
```

Then open:

`http://<pi-ip>:8088`

## Current Status

Current status:

- the UI is available
- FastAPI publishes MQTT commands
- FastAPI subscribes to state, diag, and availability
- no authentication yet
- no persistence yet

## Next Step

Run the MQTT simulator and validate the end-to-end path before touching the ESP32.

## Simulator

A minimal simulated remote is available in:

`tools/simulated_remote.py`

Run it from the Pi:

```bash
cd /opt/heliostat/remote-access/rpi
source .venv/bin/activate
python tools/simulated_remote.py
```

What it does:

- subscribes to `cmd/mode`
- subscribes to `cmd/manual`
- subscribes to `cmd/action`
- subscribes to `cmd/time`
- publishes `state`
- publishes `diag`
- publishes retained `availability`

This lets you validate:

- web UI -> FastAPI
- FastAPI -> MQTT
- MQTT -> simulated remote
- simulated remote -> MQTT state
- MQTT state -> FastAPI -> UI
