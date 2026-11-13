# Verification and remaining validation

## Completed off-hardware checks

- Recomputed SHA-256 for both recovery files and confirmed byte identity.
- Inspected a derived AVR binary/disassembly; originals were not modified.
- Compiled the Nano environment with PlatformIO 6.2.0 and Atmel AVR platform 5.3.0.
- Firmware size: 11,852 bytes flash (38.6% of 30,720) and 544 bytes static RAM (26.6% of 2,048).
- PlatformIO native tests cover known-frame CRC/round trip, invalid CRC, commissioning/sensor gates, both feedback signs, invalid measurements, deadband, final limits, travel-survey boundary/isolation rules, STOP in every motion state and encoder-stall fault latching.
- Python tests cover signed little-endian framing, logger telemetry IDs/shared sequence, invalid-distance encoding and CRC.
- `git diff --check` is used to detect whitespace errors.

Commands used:

```bash
platformio run -e nanoatmega328
platformio test -e native
python3 test/test_protocol_compatibility.py -v
```

No upload or hardware write was performed.

## Hardware-required validation checklist

- [ ] Motor stays disabled through reset, USB open/reconnect, sensor initialisation and fault entry.
- [ ] Existing logger accepts commands and produces unchanged CSV columns at approximately 10 Hz.
- [ ] VL53L0X validity/status packing matches the connected sensor across in-range and out-of-range cases.
- [ ] Positive/negative motor directions and encoder signs are measured.
- [ ] Start boost starts reliably without unacceptable overshoot or L298N heating.
- [ ] Approach PWM decelerates reliably and does not stall prematurely.
- [ ] STOP interrupts commissioning jog, homing seek, homing backoff, normal move and tracking.
- [ ] Homing reverse, initial no-movement, seek displacement/time bound, backoff stall and timeout faults are exercised.
- [ ] Homing repeatability and backed-off zero are measured over repeated cycles.
- [ ] Software min/max limits include physical clearance and reject beyond-limit commands.
- [ ] Travel survey accepts only small negative moves inside its provisional envelope and rejects all movement toward the upper stop.
- [ ] Survey STOP, 500 ms stall detection and 3000 ms timeout disable the motor as specified.
- [ ] Encoder disconnect/jam causes stall fault and immediate motor disable.
- [ ] Sensor invalid data commands no movement; one-second loss latches timeout during tracking.
- [ ] Feedback sign is experimentally established and the first correction reduces distance error.
- [ ] Correction size is bounded and tracking cannot cross either software limit.
- [ ] Deadband/median settings are validated at intended standoff and surface material.
- [ ] Non-converging tracking latches a fault; CLEAR_FAULT does not resume automatically and requires re-home.
- [ ] Automatic startup is tested only after all gates are signed off, first without auto tracking and then with it.

Automated hardware-in-the-loop tests remain desirable for STOP latency, homing timeout/stall injection, encoder-limit enforcement and startup transitions. Host tests cannot validate wiring direction, mechanical limits, sensor behavior, starting torque, L298N voltage loss or thermal performance.
