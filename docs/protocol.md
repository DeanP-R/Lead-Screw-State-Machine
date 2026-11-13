# Binary serial protocol

## Frame

All commands and responses use 10 bytes at 115,200 baud:

| Offset | Field |
|---:|---|
| 0 | Start `0xAA` |
| 1 | Command/response |
| 2 | Sequence byte |
| 3–6 | Signed 32-bit value, little-endian |
| 7 | Flags |
| 8 | CRC-8 over offsets 1–7, polynomial `0x07`, initial `0x00` |
| 9 | End `0x55` |

## Commands

| ID | Command | Value |
|---:|---|---|
| `01` | HOME | ignored |
| `02` | MOVE_ABSOLUTE | encoder counts |
| `03` | MOVE_RELATIVE | signed encoder counts |
| `04` | START_TRACKING | ignored |
| `05` | STOP_TRACKING | ignored |
| `06` | STOP | ignored |
| `07` | GET_STATUS | ignored |
| `08` | CLEAR_FAULT | ignored |
| `09` | SET_STANDOFF | millimetres |
| `0A` | ENABLE_TELEMETRY | ignored |
| `0B` | DISABLE_TELEMETRY | ignored |
| `10` | PING | ignored |

## Responses and telemetry

| ID | Response | Value | Flags |
|---:|---|---|---|
| `80` | ACK | 0 | 0 |
| `81` | NACK | 0 | reason |
| `82` | STATUS | encoder | state |
| `83` | HOME_COMPLETE | encoder | 0 |
| `84` | MOVE_COMPLETE | encoder | 0 |
| `85` | FAULT | encoder | fault code |
| `86` | DISTANCE | mm or `-1` | bit 7 valid; low 4 bits state |
| `87` | ENCODER | counts | low 4 bits state |
| `88` | SENSOR_DIAGNOSTIC | observed raw packing | observed flags (`06` valid, `03` invalid) |

When enabled, telemetry is sent every 100 ms in `86`, `87`, `88` order. All three frames share a sample sequence, which increments after the diagnostic frame. This matches `packet_sender_test_autolog.py` and its CSV association logic.

NACK reasons `01`–`06` retain their old meanings: bad CRC, unknown command, invalid state, not homed, out of range and busy. Extensions are `07` not commissioned and `08` sensor invalid. Older tools display unknown extension text but continue to parse the frame.

## Fault codes

| Code | Meaning |
|---:|---|
| `01`–`09` | Existing homing configuration/seek/backoff/zero reasons |
| `10` | Sensor initialisation failed |
| `20` | Encoder move stalled |
| `21` | Move timeout |
| `22` | Travel limit violation |
| `23` | Low-level motor command watchdog expired |
| `30` | Valid sensor data timeout |
| `31` | Tracking correction blocked by a travel limit |
| `32` | Tracking failed to converge |

These replacement fault codes are a documented extension. Response framing and telemetry fields remain compatible; scripts that do not know a new code still show its numeric flags value.
