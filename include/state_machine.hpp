#pragma once

#include <Arduino.h>

enum class LifterState : uint8_t
{
    Idle = 0, Homing = 1, Moving = 2, Tracking = 3,
    Stopping = 4, Fault = 5
};

enum class FaultCode : uint8_t
{
    None = 0x00,
    // Preserve the existing Python tool's homing reason values 0x01-0x09.
    HomingInvalidConfiguration = 0x01,
    HomingSeekNoMovement = 0x02,
    HomingSeekReverse = 0x03,
    HomingSeekTimeout = 0x04,
    HomingBackoffNoMovement = 0x05,
    HomingBackoffReverse = 0x06,
    HomingBackoffStall = 0x07,
    HomingBackoffTimeout = 0x08,
    HomingZeroing = 0x09,
    SensorInitialisation = 0x10,
    MoveStall = 0x20,
    MoveTimeout = 0x21,
    TravelLimit = 0x22,
    MotorCommandTimeout = 0x23,
    SensorTimeout = 0x30,
    TrackingAtLimit = 0x31,
    TrackingDiverging = 0x32
};

enum class RequestResult : uint8_t
{
    Accepted, Busy, InvalidState, NotHomed, OutOfRange,
    NotCommissioned, SensorInvalid
};

void initialiseStateMachine(bool sensorInitialised, unsigned long now);
void updateStateMachine(unsigned long now);
RequestResult requestHoming(uint8_t sequence, unsigned long now);
RequestResult requestMoveAbsolute(uint8_t sequence, long target, unsigned long now);
RequestResult requestMoveRelative(uint8_t sequence, long offset, unsigned long now);
RequestResult requestStartTracking(uint8_t sequence, unsigned long now);
RequestResult requestStopTracking();
void requestStop();
bool clearFault();
void setTelemetryEnabled(bool enabled);
bool isTelemetryEnabled();
LifterState getLifterState();
FaultCode getFaultCode();
