#pragma once

#include <Arduino.h>

struct SensorSample
{
    uint16_t distanceMm;
    uint8_t rangeStatus;
    bool valid;
    bool timedOut;
    unsigned long timestamp;

    SensorSample()
        : distanceMm(0), rangeStatus(0xFF), valid(false),
          timedOut(false), timestamp(0) {}
};

bool initialiseDistanceSensor();
void updateDistanceSensor(unsigned long now);
const SensorSample& getLatestSensorSample();
bool hasFreshValidDistance(unsigned long now);
uint16_t getFilteredDistance();
uint8_t getConsecutiveValidSamples();
unsigned long getLastValidDistanceTime();
int32_t getSensorDiagnosticRaw();
uint8_t getSensorDiagnosticFlags();
