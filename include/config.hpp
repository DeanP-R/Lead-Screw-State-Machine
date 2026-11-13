#pragma once

#include <Arduino.h>

namespace Config
{
    constexpr uint8_t MOTOR_EN = 9;
    constexpr uint8_t MOTOR_A = 4;
    constexpr uint8_t MOTOR_B = 7;
    constexpr uint8_t ENCODER_A = 3;
    constexpr uint8_t ENCODER_B = 6;
    constexpr uint32_t SERIAL_BAUDRATE = 115200;

    // Automatic movement remains inhibited until physical commissioning.
    constexpr bool COMMISSIONING_COMPLETE = false;
    constexpr bool MOTOR_DIRECTION_VERIFIED = false;
    constexpr bool HOMING_VERIFIED = false;
    constexpr bool TRAVEL_LIMITS_VERIFIED = false;
    constexpr bool FEEDBACK_DIRECTION_VERIFIED = false;
    constexpr bool AUTO_HOME_ON_STARTUP = false;
    constexpr bool AUTO_TRACK_ON_STARTUP = false;
    // Temporarily enable only for supervised, short direction-check jogs.
    constexpr bool COMMISSIONING_JOG_ENABLED = false;
    constexpr long COMMISSIONING_MAX_JOG_COUNTS = 100;

    constexpr int8_t HOME_SEEK_ENCODER_DIRECTION = 1;
    // +1: positive motion increases distance; -1: decreases it; 0: unknown.
    constexpr int8_t DISTANCE_CHANGE_PER_POSITIVE_MOTION = 0;

    // Conservative placeholders. They are unusable while verification is false.
    constexpr long TRAVEL_MIN_COUNTS = -5000;
    constexpr long TRAVEL_MAX_COUNTS = 0;

    constexpr uint8_t MOTOR_START_PWM = 230;
    constexpr uint8_t MOTOR_RUN_PWM = 185;
    constexpr uint8_t MOTOR_HOMING_PWM = 165;
    constexpr uint8_t MOTOR_APPROACH_PWM = 140;
    constexpr uint8_t MOTOR_MIN_PWM = 120;
    constexpr unsigned long MOTOR_START_BOOST_MS = 120;
    constexpr unsigned long MOTOR_RAMP_INTERVAL_MS = 20;
    constexpr uint8_t MOTOR_RAMP_STEP = 5;
    constexpr unsigned long MOTOR_COMMAND_TIMEOUT_MS = 8000;

    constexpr long MOVE_TOLERANCE_COUNTS = 5;
    constexpr long MOVE_MIN_PROGRESS_COUNTS = 5;
    constexpr long MOVE_DECEL_WINDOW_COUNTS = 50;
    constexpr unsigned long MOVE_STALL_TIME_MS = 500;
    constexpr unsigned long MOVE_TIMEOUT_MS = 8000;

    constexpr long HOME_BACKOFF_COUNTS = 320;
    constexpr long HOME_MAX_SEEK_COUNTS = 6000;
    constexpr long HOME_INITIAL_MOVEMENT_COUNTS = 20;
    constexpr long HOME_MIN_PROGRESS_COUNTS = 5;
    constexpr long HOME_REVERSE_LIMIT_COUNTS = 20;
    constexpr unsigned long HOME_REVERSE_GRACE_MS = 150;
    constexpr unsigned long HOME_INITIAL_STALL_MS = 1000;
    constexpr unsigned long HOME_STALL_MS = 500;
    constexpr unsigned long HOME_SEEK_TIMEOUT_MS = 5000;
    constexpr unsigned long HOME_BACKOFF_TIMEOUT_MS = 2500;
    constexpr unsigned long HOME_SETTLE_MS = 100;

    constexpr uint16_t TARGET_STANDOFF_MM = 10;
    constexpr uint16_t TRACKING_DEADBAND_MM = 2;
    constexpr uint8_t SENSOR_FILTER_SAMPLES = 5;
    constexpr uint8_t SENSOR_VALID_SAMPLES_REQUIRED = 3;
    constexpr unsigned long SENSOR_SAMPLE_INTERVAL_MS = 100;
    constexpr unsigned long SENSOR_TIMEOUT_MS = 1000;
    constexpr uint16_t SENSOR_IO_TIMEOUT_MS = 50;
    constexpr long TRACKING_COUNTS_PER_MM = 10;
    constexpr long TRACKING_MIN_CORRECTION_COUNTS = 5;
    constexpr long TRACKING_MAX_CORRECTION_COUNTS = 40;
    constexpr unsigned long TRACKING_SETTLE_MS = 150;
    constexpr uint8_t TRACKING_MAX_NON_IMPROVING_CORRECTIONS = 8;
    constexpr uint16_t TRACKING_MIN_IMPROVEMENT_MM = 1;

    constexpr unsigned long TELEMETRY_INTERVAL_MS = 100;
}
