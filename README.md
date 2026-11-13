# ACFM lead-screw lifter firmware

This branch contains the safety-oriented replacement firmware reconstructed from the older PlatformIO source, two identical flash backups, the existing Python monitor/logger and October–November 2026 CSV telemetry.

The recovered binary was **not** converted back into original C++ source. Binary analysis established the compatibility contract and exposed important differences from the old repository, most notably that the working image identifies a **VL53L0X**, not a VL6180X.

Safety defaults deliberately inhibit all automatic motion. Commissioning flags, feedback sign, homing direction and encoder limits in [config.hpp](include/config.hpp) must be established on the mechanism before tracking can be enabled.

## Documents

- [Reconstruction evidence and implementation plan](docs/reconstruction.md)
- [Architecture, state machine, motor, homing and tracking](docs/architecture.md)
- [Binary serial protocol and fault codes](docs/protocol.md)
- [Configuration reference and commissioning procedure](docs/commissioning.md)
- [Verification results and validation checklist](docs/validation.md)

## Build and host tests

```bash
platformio run -e nanoatmega328
platformio test -e native
python3 test/test_protocol_compatibility.py -v
```

No upload is part of this work. Do not run an upload target until commissioning has completed and the configuration has been independently reviewed.
