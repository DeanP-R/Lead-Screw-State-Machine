# Lead-Screw Lifter Firmware

This repository contains the firmware for a motorised lead-screw lifter used to position an ACFM probe. The firmware runs on an Arduino Nano (ATmega328P) and controls a DC motor through an H-bridge using encoder feedback for position measurement and a VL6180X time-of-flight sensor for stand-off measurement.

The project forms the low-level embedded controller for the probe lifter. It is responsible for controlling the motor, monitoring the encoder, reading the distance sensor and carrying out functions such as homing and stand-off tracking. Higher-level decisions and command generation are intended to be handled by a ROS 2 node running on the main computer.

## Features

Current functionality includes:

- Encoder-based homing using the upper mechanical stop.
- Software zero established after backing away from the stop.
- Quadrature encoder position tracking using interrupts.
- Closed-loop stand-off tracking using a VL6180X distance sensor.
- Modular source structure with separate motor, encoder, sensor and behaviour modules.
- Configurable parameters stored in a single configuration header.

## Repository structure
- `platformio.ini` — build and upload configuration for PlatformIO.
- `src/` — application source files:
	- [main.cpp](src/main.cpp) - initialisation and main loop.
	- [homing.cpp](src/homing.cpp) - homing state machine and fault handling.
	- [encoder.cpp](src/encoder.cpp) - encoder interrupt handler and position API.
	- [motor_driver.cpp](src/motor_driver.cpp) - motor direction, PWM control and pulse helpers.
	- [vl6180x_driver.cpp](src/vl6180x_driver.cpp) - sensor initialisation and range read.
	- [tracking.cpp](src/tracking.cpp) - standoff control logic.
- `include/` — public headers (`.hpp`) that define interfaces and configuration:
	- [include/config.hpp](include/config.hpp)
	- [include/encoder.hpp](include/encoder.hpp)
	- [include/motor_driver.hpp](include/motor_driver.hpp)
	- [include/vl6180x_driver.hpp](include/vl6180x_driver.hpp)
	- [include/tracking.hpp](include/tracking.hpp)
	- [include/homing.hpp](include/homing.hpp)


## Hardware

The firmware has been developed for:

- Arduino Nano (ATmega328P)
- DC motor with quadrature encoder
- L298N motor driver
- VL6180X time-of-flight sensor
- Lead-screw linear actuator

The encoder provides position feedback while the VL6180X measures the distance between the probe and the surface.

## Operation

### Startup

During startup the firmware:

1. Initialises serial communication.
2. Initialises I²C.
3. Configures the motor driver.
4. Configures the encoder interrupt.
5. Initialises the VL6180X.
6. Executes the homing routine.

If the distance sensor cannot be initialised, the firmware stops and reports a fault over the serial port.

### Homing

The homing routine establishes a repeatable software reference position.

The sequence is:

1. Drive the lifter towards the upper mechanical stop.
2. Monitor encoder movement.
3. Detect the stop when encoder movement ceases.
4. Stop the motor.
5. Back away from the stop by a fixed encoder distance.
6. Reset the encoder position to zero.

If the homing sequence cannot be completed within the configured limits, the firmware reports a fault and aborts the procedure.

### Stand-off Tracking

Tracking uses the VL6180X to maintain the required probe stand-off.

The controller repeatedly:

1. Reads the distance sensor.
2. Compares the measured distance with the target.
3. Commands a short motor movement if the measurement lies outside the configured tolerance.
4. Holds position once the measurement is within tolerance.

## Configuration

All hardware mappings and configurable parameters are defined in `include/config.hpp`.

Examples include:

- motor pins
- encoder pins
- PWM values
- pulse duration
- target stand-off
- stand-off tolerance
- homing timeout
- encoder movement thresholds
- homing backoff distance

## Building

The project is built using PlatformIO.

Compile:

```bash
platformio run
```

Compile and upload:

```bash
platformio run --target upload
```

## Current Limitations

The current implementation uses encoder stall detection to locate the upper mechanical stop. This approach works without additional sensors but cannot distinguish between reaching the intended stop and an unexpected obstruction elsewhere in the mechanism.

The VL6180X has also shown noticeable measurement variation at short stand-off distances. Alternative distance sensors are currently being investigated for improved tracking accuracy.

#TODO

- Introduce a finite state machine for firmware control.
- Add a binary serial protocol for communication with ROS 2.
- Implement encoder position moves.
- Add fault reporting over the serial interface.
- Replace the temporary tracking implementation with the state-machine version.
- Investigate alternative stand-off sensors.
- Consider adding current sensing or a limit switch to improve homing robustness.
