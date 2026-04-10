# Remote Access Architecture

## Goal

Allow the heliostat `remote` ESP32 to be controlled over the Internet while keeping:

- the tracking logic on the ESP32 `remote`
- the Raspberry Pi in the workshop as the central server
- user access from a laptop or phone through the existing VPN
- no direct inbound access to the ESP32 on the field network

The field setup is expected to work behind a mobile access point or hotspot.

## High-Level Design

Three actors are involved:

1. `remote` ESP32 on the field
2. Raspberry Pi server in the workshop
3. user interface from laptop or phone

The communication model is:

- `remote ESP32 -> Raspberry Pi`: outbound connection only
- `browser/app -> Raspberry Pi`: user access through VPN
- `Raspberry Pi`: central coordination point

This avoids any need to reach the ESP32 directly from the Internet.

## Why This Architecture

The ESP32 on the field may be behind:

- a phone hotspot
- a mobile router
- another NATed network

In that situation, the robust pattern is:

- the ESP32 initiates and maintains a connection to the workshop
- the workshop server stores the current state and forwards commands
- the user talks only to the workshop server

This keeps the ESP32 simple and avoids port forwarding on the field side.

## Proposed Stack

### ESP32 Remote

Responsibilities:

- connect to Wi-Fi on the field
- connect to the workshop server using MQTT over TLS
- subscribe to control commands
- publish state and diagnostics
- keep heliostat logic local:
  - manual movement
  - target capture
  - auto tracking
  - local safety / timeouts

### Raspberry Pi

Responsibilities:

- host the MQTT broker
- host a lightweight web UI
- translate user actions into MQTT commands
- receive state and diagnostics from the ESP32
- remain reachable from the user through the existing VPN

Suggested software:

- Mosquitto for MQTT
- FastAPI for the web UI and API
- systemd services for process management

### User Side

Responsibilities:

- connect to the Raspberry Pi through the existing VPN
- open the heliostat web UI from a laptop or phone
- control manual mode, capture, auto mode, recenter, and diagnostics

## Networking Model

### Workshop Side

The Raspberry Pi stays in the workshop and is reachable from the user through VPN.

The Pi exposes:

- a web UI on a dedicated port
- an MQTT broker endpoint reachable by the ESP32

### Field Side

The ESP32 only needs:

- Internet access through the hotspot
- outbound access to the workshop MQTT endpoint

No inbound route to the ESP32 is required.

## Reliability Principles

To keep the bug surface low:

- keep the heliostat state machine on the ESP32
- keep the Pi stateless where possible
- use MQTT with simple retained state topics
- treat the web UI as a client, not as the source of truth

The source of truth should be:

- current mode and physical state on the ESP32

The Pi should be:

- a command relay
- a state observer
- a UI host

## Safety / Failure Behavior

If the remote Internet link drops:

- the ESP32 should keep its current safe mode
- manual commands should time out
- auto mode should continue only if it is safe to do so
- the Pi should mark the remote as offline after a timeout

If the Pi or MQTT broker becomes unavailable:

- the ESP32 should detect the disconnect
- reconnect periodically
- keep a safe local behavior while disconnected

## Development Strategy

Keep this work in the same git repository but isolated in a dedicated area:

- firmware remains in `src/`
- remote access work lives in `remote-access/`

Suggested branch split:

- tracking work on one branch
- remote access work on another branch

Suggested implementation order:

1. define the protocol between Pi and ESP32
2. build the Pi MQTT + UI skeleton
3. test the Pi locally with fake messages
4. add MQTT support to the ESP32 `remote`
5. replace or complement ESP-NOW with the new path

## Out of Scope For First Iteration

To keep the first version manageable, do not start with:

- user accounts
- database storage
- camera/video streaming
- advanced dashboards
- multiple remotes
- cloud deployment

The first version should only prove:

- manual remote control over the Internet
- target capture
- auto mode enable/disable
- state and diagnostics visibility
