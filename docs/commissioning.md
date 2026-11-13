# Configuration and hardware commissioning

## Configuration reference

All values are in `include/config.hpp`.

| Group | Parameters | Purpose |
|---|---|---|
| Safety gates | `COMMISSIONING_COMPLETE`, `*_VERIFIED` | Compile-time evidence gates; all false by default |
| Automatic startup | `AUTO_HOME_ON_STARTUP`, `AUTO_TRACK_ON_STARTUP` | Both false by default; auto tracking also requires every gate |
| Direction | `HOME_SEEK_ENCODER_DIRECTION`, `DISTANCE_CHANGE_PER_POSITIVE_MOTION` | Must be measured, never assumed |
| Limits | `TRAVEL_MIN_COUNTS`, `TRAVEL_MAX_COUNTS` | Normal-motion envelope after home |
| Commissioning jog | `COMMISSIONING_JOG_ENABLED`, `COMMISSIONING_MAX_JOG_COUNTS` | Temporary supervised short relative jog; default disabled |
| PWM | `MOTOR_START_PWM`, `MOTOR_RUN_PWM`, `MOTOR_HOMING_PWM`, `MOTOR_APPROACH_PWM`, `MOTOR_MIN_PWM` | Boost, cruise, homing and approach duty |
| Motion safety | boost/ramp, command/move timeout, tolerance, progress and decel window | Start behavior and watchdogs |
| Homing | seek/backoff counts, reverse limits, progress windows and phase timeouts | Bounded stall-based reference sequence |
| Sensor | sample/I/O/validity timeout and median sample count | VL53L0X acquisition and validation |
| Tracking | target, deadband, counts/mm, correction bounds, settle and convergence limits | Step controller |
| Telemetry | `TELEMETRY_INTERVAL_MS` | 100 ms compatibility cadence |

## Progressive commissioning procedure

Use supports that prevent the probe/carriage falling, provide an accessible power disconnect, and keep a person at STOP/power. Begin with automatic startup flags false. Opening USB serial can reset the Nano; a reset must be tested with the mechanism clear.

1. **Mechanical/electrical inspection:** remove overtension, verify free lead-screw/belt motion, alignment, connectors, encoder pull-ups, L298N enable jumper/wiring, supply voltage and current capability. Measure motor voltage under starting load and monitor L298N temperature.
2. **No-motion boot:** build with every gate and commissioning jog false. Power/reset and reconnect USB several times. Confirm no carriage motion, IDLE or the expected sensor fault, valid STOP, and no automatic restart after CLEAR_FAULT.
3. **Sensor-only validation:** enable telemetry. Verify VL53L0X distance, `86/87/88` ordering, approximately 10 Hz cadence, invalid/out-of-range handling and stationary encoder. Repeat at intended standoff and relevant surfaces; do not reuse the earlier 89 mm noise result as the operating result.
4. **Short direction jog:** physically center the carriage. Temporarily enable only `COMMISSIONING_JOG_ENABLED`; keep auto flags false. Use very small relative commands in both signs, record physical direction and encoder sign, test STOP during motion, then disable the jog feature.
5. **PWM characterization:** with conservative travel, test start PWM/duration, run and approach PWM across expected orientation/load. Record failures, voltage and encoder latency. Fix mechanical resistance first; assess a MOSFET H-bridge if L298N voltage loss limits torque.
6. **Homing direction:** set `HOME_SEEK_ENCODER_DIRECTION` from the measured encoder sign and set `MOTOR_DIRECTION_VERIFIED=true`. Supervise one home. Test STOP during seek and backoff, reverse-motion detection, no-movement behavior and timeout. Confirm it backs away and zeroes only after success.
7. **Travel survey:** from the verified home, measure the usable range without contacting the opposite stop. Set conservative min/max counts with clearance. Set `HOMING_VERIFIED` and `TRAVEL_LIMITS_VERIFIED` only after repeated tests. Attempt commands just inside and outside both limits.
8. **Feedback sign:** at a safe mid-travel point, make one small positive encoder move and measure whether VL53L0X distance increases or decreases. Set `DISTANCE_CHANGE_PER_POSITIVE_MOTION` to `+1` or `-1`; repeat to confirm and then set `FEEDBACK_DIRECTION_VERIFIED=true`.
9. **Manual tracking:** set target/deadband from measurements at the actual surface. Start tracking manually. Verify the first correction reduces error, invalid readings stop motion, all corrections stay inside limits, STOP is immediate, and a non-improving sequence faults.
10. **Endurance and restart:** test orientation/load extremes, cable motion, sensor dropout, encoder disconnect, motor jam, serial disconnect/reconnect, controller reset and repeated home/track cycles. Review logs and temperature.
11. **Automatic startup:** only after sign-off set `COMMISSIONING_COMPLETE=true`. Enable `AUTO_HOME_ON_STARTUP`, then separately `AUTO_TRACK_ON_STARTUP`. Re-run reset/USB/power-loss tests. Keep a maintenance build/profile with automatic flags false.

Record the exact tested firmware commit and configuration values. Compilation and simulation alone do not establish hardware reliability.
