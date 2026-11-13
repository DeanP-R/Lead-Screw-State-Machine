# Replacement firmware architecture

## Components

- `main.cpp`: safe initialisation and packet polling.
- `protocol` / `comms`: pure frame codec plus resynchronising serial transport.
- `motor_driver`: H-bridge direction, start boost, PWM transition and command watchdog.
- `encoder`: quadrature decoding on INT1 and PCINT22.
- `distance_sensor`: VL53L0X sampling, validation, median filter and raw diagnostics.
- `homing`: cooperative seek, settle, backoff and zero phases.
- `control_logic`: pure commissioning gates, feedback direction and bounded correction calculation.
- `state_machine`: command arbitration, travel limits, motion watchdogs, tracking and latched faults.

## Startup and state machine

The bridge enable latch is driven low before its pin becomes an output. Encoder, serial and sensor are then initialised, followed by the state machine. A sensor initialisation failure enters latched FAULT with the motor off.

Protocol-visible states remain compatible with the logger:

| Value | State | Motion |
|---:|---|---|
| 0 | IDLE / ready | Off |
| 1 | HOMING | Bounded seek or backoff |
| 2 | MOVING | Encoder-target move |
| 3 | TRACKING | Bounded correction only |
| 4 | STOPPING | Reserved for compatibility |
| 5 | FAULT | Off and latched |

With default configuration, successful initialisation enters IDLE and cannot move automatically. When every commissioning gate is deliberately enabled, startup can request non-blocking homing, wait for a valid sensor streak, then enter tracking. STOP cancels pending automatic tracking.

## Motor control

Every move begins with configurable `MOTOR_START_PWM`. Once meaningful encoder progress is seen, PWM changes to the requested run value. Near a normal target it uses `MOTOR_APPROACH_PWM`. A command-duration watchdog inside the driver disables the H-bridge even if higher-level control fails; state-level stall and move timeouts latch faults.

The boost is only one mitigation. Belt tension, alignment, supply voltage under load, L298N voltage drop/thermal limits, motor current and gearbox/lead-screw friction require physical checks. An L298N can materially reduce available motor voltage and has no current feedback in this design; a modern appropriately rated MOSFET bridge is a commissioning recommendation, not assumed by the firmware.

## Homing

Homing is permitted only after motor/encoder direction has been verified. It is cooperative, so the serial parser continues to handle STOP.

1. Seek in configured encoder direction.
2. Reject sustained reverse counts.
3. Bound seek by both elapsed time and displacement from its start.
4. Interpret loss of encoder progress as the existing stall-reference mechanism.
5. Stop and settle.
6. Back off in the opposite direction by a configured count.
7. Reject no movement, reverse movement, backoff stall and timeout.
8. Stop, settle and set the backed-off position to zero.

This retains the existing no-limit-switch mechanism. A hard stop is used only to establish the reference, never as a normal travel endpoint. Adding a limit switch or current sensing would improve fault discrimination.

## Tracking algorithm

The controller intentionally uses explainable step control rather than PID:

1. Sample at 100 ms and reject timeout, nonzero VL53L0X range status, or zero distance.
2. Maintain a five-sample median and require three consecutive valid readings.
3. Stop immediately on an invalid sample; latch SENSOR_TIMEOUT if valid data does not return within one second.
4. Compute `target - filtered_distance`. Hold inside the deadband.
5. Convert error to encoder counts using the physically commissioned feedback sign.
6. Clamp each correction between configured minimum/maximum step sizes and mechanical limits.
7. Move using encoder feedback, stall/timeout checks and approach PWM.
8. Allow a settle interval, then evaluate the next new sensor sample.
9. Latch TRACKING_DIVERGING after the configured number of non-improving corrections.

Unknown feedback direction, unverified limits, lack of home or invalid sensor data prevents tracking from starting.

## Supervised travel survey

Travel survey is a separate commissioning-only relative-move path. It requires an existing valid home, verified motor direction and verified homing, while final travel limits and completed commissioning must still be false. Automatic homing and tracking must be disabled.

Each command must be a small negative move away from backed-off home zero. Positive moves are rejected, targets are restricted to a configurable provisional envelope, and the measured encoder position is checked throughout motion. Survey moves use the normal encoder progress detector with survey-specific stall and timeout values. STOP follows the same immediate cancellation path as other motion. Enabling final verified limits removes the survey path; it cannot widen or bypass the normal envelope.

## Recovery behavior

Critical faults disable the motor and latch FAULT. STOP always disables motion but does not clear a critical fault. CLEAR_FAULT is accepted only in FAULT; a sensor-initialisation fault must successfully reinitialise the sensor. Recovery invalidates home, so normal movement/tracking requires re-homing. Automatic startup does not restart after a runtime fault.
