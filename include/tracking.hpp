#pragma once

#include <Arduino.h>

void resetTrackingMonitor();
bool setTrackingStandoff(uint16_t millimetres);
uint16_t getTrackingStandoff();
bool recordTrackingError(uint16_t absoluteErrorMm);
