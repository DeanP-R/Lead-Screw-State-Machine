# Firmware reconstruction evidence and plan

## Preservation

The recovered files remain unchanged in `/home/ubuntu22/acfm_lifter_recovery_20261113T124944Z/`:

| File | SHA-256 |
|---|---|
| `acfm_lifter_backup_read1.hex` | `29bf59de5dd75da46a1235a6bcf8e76f71e49af335968c2e82be9275a32418d2` |
| `acfm_lifter_backup_read2.hex` | `29bf59de5dd75da46a1235a6bcf8e76f71e49af335968c2e82be9275a32418d2` |

`cmp` reports the files byte-for-byte identical. They are 77,836-byte Intel HEX files representing 32,768 flash bytes. Analysis used derived files under `/tmp`; it did not edit either backup. The recovery report records an ATmega328P signature `1E 95 0F`, STK500v1 Arduino bootloader and 57,600 baud.

## Binary analysis

The Intel HEX was converted to a flat 32,768-byte analysis copy and inspected with AVR GNU binutils 2.26 (`avr-objdump`, `avr-objcopy`) and `strings`. The application occupies the lower flash area and a traditional Nano boot section is visible near `0x7800`.

### Confirmed findings

- The image contains `VL53L0X not detected`, VL53L0X silicon-revision strings and `LIFTER CONTROL READY` at flash offsets around `0x4B1A`–`0x4B70`. The recovered implementation therefore used a VL53L0X-family driver.
- Frames are ten bytes: `AA`, command, sequence, signed little-endian 32-bit value, flags, CRC, `55`.
- The CRC routine uses polynomial `0x07`, initial value zero, across bytes 1–7.
- Response identifiers `0x80`–`0x88` are present and match ACK, NACK, STATUS, HOME_COMPLETE, MOVE_COMPLETE, FAULT, DISTANCE, ENCODER and SENSOR_DIAGNOSTIC in the Python logger.
- Telemetry emits distance (`0x86`), encoder (`0x87`) and diagnostic (`0x88`) with one shared sample sequence, then increments the sequence.
- The telemetry interval comparison is 100 ms, consistent with the approximately 10 Hz CSV logs.
- Invalid distance is encoded as signed `-1`; valid distance sets flags bit `0x80`.
- Commands `0x0A` and `0x0B` enable and disable telemetry.
- A 1,000 ms sensor-related timeout constant is used in the telemetry/sensor path.
- The binary contains manual relative-motion and timeout/stall diagnostic text.

### Evidence-supported inferences

- CSV diagnostics commonly contain raw `0x01B40000` with flags `0x06`; invalid samples include low-byte status values such as `0x07`, `0x0D`, `0x06` and `0x0B` with flags `0x03`. The replacement preserves this observed packing shape, but the original symbolic bit names cannot be recovered.
- Constants `200` and `150` occur in motor-related code and are likely PWM values, but stripped machine code does not provide reliable original variable names or full intent.
- Control-flow structure and behavior can be inferred from disassembly, but original C++ types, comments, identifiers and exact source layout cannot be recovered.

## Older repository versus recovered implementation

| Area | Earlier repository | Recovered image / logs | Replacement |
|---|---|---|---|
| ToF sensor | Adafruit VL6180X | VL53L0X strings and diagnostics | Pololu VL53L0X driver |
| Telemetry | No periodic distance/encoder/diagnostic stream | 10 Hz `0x86/0x87/0x88` triplets | Same IDs, cadence, ordering and sequence |
| Homing | Blocking loops and delays | Source unavailable; motion diagnostics present | Promptly interruptible phased state machine |
| Relative move | Blocking motor pulses | Timeout/stall evidence | Continuous ramped command with encoder target and watchdogs |
| Absolute move | Stub | Command exists in Python contract | Implemented with homing and limit gates |
| Tracking | One raw reading and fixed pulse; assumed sign | Command accepted in working protocol | Median filter, validity streak, explicit commissioned sign, bounded corrections |
| Travel limits | None | Not proven from binary | Mandatory verified encoder limits |
| Startup motion | Repository comments implied homing | Not safely provable | Motor off; automatic motion disabled by default |
| Fault recovery | State clear with few invariants | Fault response confirmed | Latched fault, STOP-safe, explicit CLEAR_FAULT, home invalidation |

## Applicability of the earlier audit

The findings about blocking homing, absent software limits, assumed feedback direction, fixed pulse PWM, absolute-move stub, and missing telemetry apply to the **older source**, not automatically to the recovered firmware. Binary/log evidence disproves the old sensor and telemetry findings. The recovered source is unavailable, so its complete homing, limit, startup and tracking safety cannot be verified; the replacement addresses those requirements independently rather than claiming those bugs existed in the recovered image.

## Reconstruction and implementation plan

1. Treat the recovered framing, response IDs, telemetry triplet/cadence and the Python logger as the compatibility authority.
2. Replace the stale VL6180X integration with VL53L0X while retaining the observed diagnostic wire shape.
3. Separate pure protocol/control decisions from Arduino hardware code so feedback sign, invalid data, framing and bounds are host-testable.
4. Make motor, homing and movement cooperative and interruptible; add start boost, ramp/approach PWM, encoder progress checks and timeouts.
5. Require a valid home and verified software limits for normal movement; require all commissioning gates plus a known sensor feedback sign for tracking.
6. Filter sensor samples, require a valid streak, stop on any invalid sample, fault on timeout, and limit every tracking correction.
7. Latch critical faults, make STOP immediately disable the bridge in all states, and require explicit recovery.
8. Compile for Nano, run host protocol/control tests, verify Python wire compatibility, then stop before upload.

This plan is implemented on branch `reconstruction/safe-firmware`; physical commissioning remains outstanding.
