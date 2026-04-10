# MQTT Protocol Draft

## Goal

Define a simple and stable protocol between:

- the Raspberry Pi workshop server
- the field `remote` ESP32
- the web UI

The protocol should stay minimal for the first implementation.

## Naming

Base topic:

`heliostat/remote1`

The suffix `remote1` allows future extension to multiple devices without redesigning the protocol.

## Topic Layout

### Commands To The ESP32

- `heliostat/remote1/cmd/mode`
- `heliostat/remote1/cmd/manual`
- `heliostat/remote1/cmd/action`
- `heliostat/remote1/cmd/time`

### State From The ESP32

- `heliostat/remote1/state`
- `heliostat/remote1/diag`
- `heliostat/remote1/availability`

## Command Topics

### `heliostat/remote1/cmd/mode`

Purpose:

- set the main operating mode

Payload:

```json
{
  "mode": "manual"
}
```

Allowed values:

- `manual`
- `captured`
- `auto`

Notes:

- `captured` means target is known but auto tracking is not active
- the ESP32 remains responsible for validating illegal transitions

### `heliostat/remote1/cmd/manual`

Purpose:

- drive the mirror manually

Payload:

```json
{
  "pan_rate": 0.35,
  "tilt_rate": -0.20,
  "precision": false,
  "seq": 1234
}
```

Fields:

- `pan_rate`: float from `-1.0` to `1.0`
- `tilt_rate`: float from `-1.0` to `1.0`
- `precision`: boolean
- `seq`: optional monotonic command sequence number

Notes:

- this is a rate command, not an absolute angle command
- if no fresh manual command arrives within a timeout, both rates go back to zero

### `heliostat/remote1/cmd/action`

Purpose:

- trigger one-shot actions

Payload examples:

```json
{"action": "capture_target"}
```

```json
{"action": "recenter"}
```

```json
{"action": "print_diag"}
```

Allowed values for `action`:

- `capture_target`
- `recenter`
- `print_diag`

### `heliostat/remote1/cmd/time`

Purpose:

- synchronize UTC time and time scale

Payload:

```json
{
  "unix_utc": 1775803200,
  "time_scale": 1.0
}
```

Fields:

- `unix_utc`: integer UTC epoch seconds
- `time_scale`: float, usually `1.0`

Notes:

- this is the remote-access equivalent of the current serial time-setting path

## State Topics

### `heliostat/remote1/state`

Purpose:

- publish the latest operational state of the ESP32

Payload:

```json
{
  "mode": "auto",
  "pan_deg": 24.675,
  "tilt_deg": 36.690,
  "pan_us": 820,
  "tilt_us": 1285,
  "sun_time_ok": true,
  "target_ok": true,
  "remote_utc": 1775803262,
  "auto_enabled": true,
  "precision": false,
  "wifi_ok": true,
  "mqtt_ok": true
}
```

Notes:

- this topic should usually be retained
- it is the main source for UI state

### `heliostat/remote1/diag`

Purpose:

- publish diagnostic snapshots or explicit debug events

Payload:

```json
{
  "event": "tracking_diag",
  "mode": "manual",
  "drift_s": 420,
  "auto_ref_pan_deg": 31.250,
  "actual_pan_deg": 32.800,
  "delta_pan_deg": 1.550,
  "auto_ref_tilt_deg": 40.000,
  "actual_tilt_deg": 39.200,
  "delta_tilt_deg": -0.800
}
```

Notes:

- not necessarily retained
- useful for field debugging and support

### `heliostat/remote1/availability`

Purpose:

- publish online/offline status using MQTT Last Will and Testament

Payload values:

- `online`
- `offline`

Notes:

- retained topic
- set by MQTT session lifecycle

## Timing Rules

### Manual Command Timeout

If `cmd/manual` is not refreshed within a short timeout:

- `pan_rate = 0`
- `tilt_rate = 0`

Suggested timeout:

- `250 ms` to `500 ms`

### State Publish Rate

Suggested initial rate:

- `2 Hz` to `5 Hz`

This is enough for UI feedback without overloading the link.

### Diagnostics Publish Rate

Suggested behavior:

- publish on change
- publish on explicit debug action
- avoid constant spam

## Retained Messages

Recommended retained topics:

- `heliostat/remote1/state`
- `heliostat/remote1/availability`

Command topics should generally not be retained, except if a specific command type later needs it.

## First-Version Rules

The first implementation should keep these constraints:

- one remote only
- one operator at a time
- no persistent historical database
- no arbitration between multiple control clients

If multiple UI sessions are open, the simplest policy is:

- last command wins

## Mapping To Current Firmware

Current firmware concepts already map well to MQTT:

- manual stick input -> `cmd/manual`
- `X` capture -> `cmd/action`
- `Y` auto toggle -> `cmd/mode`
- serial time set -> `cmd/time`
- runtime logs -> `diag`
- periodic status -> `state`

This means the current code can evolve rather than be rewritten from scratch.
