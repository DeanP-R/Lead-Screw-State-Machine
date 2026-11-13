#pragma once

#include <Arduino.h>

enum class HomingResult : uint8_t
{
    InProgress = 0xFF,
    Success = 0x00,
    InvalidConfiguration = 0x01,
    SeekNoMovement = 0x02,
    SeekReverseMovement = 0x03,
    SeekTimeout = 0x04,
    BackoffNoMovement = 0x05,
    BackoffReverseMovement = 0x06,
    BackoffStall = 0x07,
    BackoffTimeout = 0x08,
    ZeroingFailed = 0x09,
    Cancelled = 0x0A
};

bool startHoming(unsigned long now);
HomingResult updateHoming(unsigned long now);
void cancelHoming();
bool isHomingActive();
bool isHomed();
void invalidateHome();
