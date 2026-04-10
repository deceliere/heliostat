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

This is only a scaffold:

- the UI is static
- MQTT bridge is not implemented yet
- no authentication yet
- no persistence yet

## Next Step

Add MQTT publish/subscribe support in the FastAPI app and connect the UI to real state.
