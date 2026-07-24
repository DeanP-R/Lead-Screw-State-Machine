#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
namespace Config
{
    // Motor driver pins
    constexpr uint8_t MOTOR_EN = 9;
    constexpr uint8_t MOTOR_A  = 4;
    constexpr uint8_t MOTOR_B  = 7;

    // Encoder pins
    constexpr uint8_t ENCODER_A = 3;
    constexpr uint8_t ENCODER_B = 6;

    // Motor settings
    constexpr uint8_t MOTOR_PWM = 200;
    constexpr uint8_t MOTOR_PWM_SLOW = 150;
    constexpr unsigned long MOTOR_PULSE_MS = 10;

    // Distance-control settings
    constexpr uint8_t TARGET_MM = 10;
    constexpr uint8_t TOLERANCE_MM = 2;

    // Homing settings
    constexpr unsigned long HOME_STALL_TIME_MS = 500;
    constexpr unsigned long HOME_TIMEOUT_MS = 10000;
    constexpr long HOME_BACKOFF_TIMEOUT_MS = 2000;
    constexpr long HOME_BACKOFF_COUNTS = 32;
    constexpr long HOME_MIN_MOVEMENT_COUNTS = 5;

    // Delay allowing mechanical load and encoder signals to settle.
    constexpr unsigned long HOME_SETTLE_TIME_MS = 100;

    // How often diagnostic information is printed.
    constexpr unsigned long HOME_DEBUG_INTERVAL_MS = 100;

    // Maximum encoder movement in the unexpected direction before
    // treating it as a direction or wiring fault.
    constexpr long HOME_REVERSE_MOVEMENT_LIMIT = 20;
}
