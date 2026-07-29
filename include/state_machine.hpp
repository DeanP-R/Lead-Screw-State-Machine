#pragma once

#include <Arduino.h>

enum class LifterState : uint8_t
{
    Idle             = 0x00,
    Homing           = 0x01,
    MovingAbsolute   = 0x02,
    MovingRelative   = 0x03,
    Tracking         = 0x04,
    Fault            = 0x05
};

void initialiseStateMachine();

void updateStateMachine();

LifterState getLifterState();

bool requestHoming(uint8_t sequence);

bool requestMoveAbsolute(
    uint8_t sequence,
    int32_t targetPosition
);

bool requestMoveRelative(
    uint8_t sequence,
    int32_t movement
);

bool requestStartTracking(uint8_t sequence);

bool requestStopTracking(uint8_t sequence);

void requestStop();

void clearFault();