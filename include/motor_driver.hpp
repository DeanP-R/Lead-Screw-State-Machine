#pragma once

#include <Arduino.h>

enum class MotorDirection : int8_t { Negative = -1, Stopped = 0, Positive = 1 };

void initialiseMotor();
void commandMotor(MotorDirection direction, uint8_t runPwm, unsigned long now);
void updateMotor(unsigned long now);
void finishMotorStartBoost();
void stopMotor();
bool isMotorRunning();
MotorDirection getMotorDirection();
uint8_t getAppliedPwm();
unsigned long getMotorCommandStartTime();
