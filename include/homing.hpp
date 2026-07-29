#pragma once

#include <stdint.h>

enum class HomingResult : uint8_t
{
    Success = 0x00,

    InvalidConfiguration = 0x01,

    SeekNoMovement = 0x02,
    SeekReverseMovement = 0x03,
    SeekTimeout = 0x04,

    BackoffNoMovement = 0x05,
    BackoffReverseMovement = 0x06,
    BackoffStall = 0x07,
    BackoffTimeout = 0x08,

    ZeroingFailed = 0x09
};

/**
 * Perform the complete blocking homing sequence.
 *
 * @return HomingResult describing success or the failure reason.
 */
HomingResult homing();

/**
 * Return whether the axis has completed homing successfully.
 */
bool isHomed();

/**
 * Mark the axis as unhomed.
 */
void clearHomedState();