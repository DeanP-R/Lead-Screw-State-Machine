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
    // Grace period after starting a seek/backoff move, during which
    // small reverse encoder noise (gear backlash, stop rebound) is
    // tolerated rather than treated as a reverse-movement fault.
    static constexpr unsigned long HOME_REVERSE_CHECK_GRACE_MS = 150;
    constexpr unsigned long HOME_STALL_TIME_MS = 500;
    constexpr unsigned long HOME_TIMEOUT_MS = 5000;
    constexpr long HOME_BACKOFF_TIMEOUT_MS = 2000;
    constexpr long HOME_BACKOFF_COUNTS = 320;
    constexpr long HOME_MIN_MOVEMENT_COUNTS = 5;
    
    static constexpr unsigned long HOME_INITIAL_STALL_TIME_MS = 1000;
    
    static constexpr int8_t HOME_UPWARD_ENCODER_DIRECTION = 1;

    /*
    * Minimum net movement required before deciding the axis genuinely
    * travelled during the initial seek.
    *
    * This should be greater than encoder jitter at the mechanical stop.
    */
    static constexpr long HOME_INITIAL_MOVEMENT_COUNTS = 20;

    // Delay allowing mechanical load and encoder signals to settle.
    constexpr unsigned long HOME_SETTLE_TIME_MS = 100;

    // How often diagnostic information is printed.
    constexpr unsigned long HOME_DEBUG_INTERVAL_MS = 100;

    // Maximum encoder movement in the unexpected direction before
    // treating it as a direction or wiring fault.
    constexpr long HOME_REVERSE_MOVEMENT_LIMIT = 20;

    // MoveRelative / MoveAbsolute settings
    // How close (in encoder counts) is close enough to call a move complete.
    static constexpr long MOVE_TOLERANCE_COUNTS = 5;

    // Safety cutoff for a single commanded move, in case the mechanism
    // stalls or the target is unreachable.
    static constexpr unsigned long MOVE_TIMEOUT_MS = 8000;

    // If less than MOVE_MIN_PROGRESS_COUNTS of progress toward the
    // target has been made within MOVE_STALL_TIME_MS, stop rather than
    // continuing to drive into whatever is blocking the mechanism
    // (e.g. a target requested past the physical end of travel).
    static constexpr long MOVE_MIN_PROGRESS_COUNTS = 5;
    static constexpr unsigned long MOVE_STALL_TIME_MS = 500;

    // Message packet params

    constexpr uint32_t SERIAL_BAUDRATE = 115200;
    constexpr uint8_t PACKET_LENGTH = 10;

    constexpr uint8_t START_BYTE = 0xAA;
    constexpr uint8_t END_BYTE   = 0x55;

    constexpr uint8_t CRC_POLYNOMIAL = 0x07;
    constexpr uint8_t CRC_INITIAL    = 0x00;

    // Packet byte indices
    constexpr uint8_t START_INDEX = 0;
    constexpr uint8_t COMMAND_INDEX = 1;
    constexpr uint8_t SEQUENCE_INDEX = 2;
    constexpr uint8_t VALUE_INDEX = 3;
    constexpr uint8_t FLAGS_INDEX = 7;
    constexpr uint8_t CRC_INDEX = 8;
    constexpr uint8_t END_INDEX = 9;
}