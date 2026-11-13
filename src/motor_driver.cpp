#include "motor_driver.hpp"
#include "config.hpp"

namespace
{
    MotorDirection direction = MotorDirection::Stopped;
    uint8_t requestedPwm = 0;
    uint8_t appliedPwm = 0;
    unsigned long commandStart = 0;
    unsigned long lastRamp = 0;

    void applyDirection(MotorDirection requested)
    {
        if (requested == MotorDirection::Positive)
        {
            digitalWrite(Config::MOTOR_A, LOW);
            digitalWrite(Config::MOTOR_B, HIGH);
        }
        else if (requested == MotorDirection::Negative)
        {
            digitalWrite(Config::MOTOR_A, HIGH);
            digitalWrite(Config::MOTOR_B, LOW);
        }
        else
        {
            digitalWrite(Config::MOTOR_A, LOW);
            digitalWrite(Config::MOTOR_B, LOW);
        }
    }
}

void initialiseMotor()
{
    // EN is driven low before either direction pin can energise the bridge.
    digitalWrite(Config::MOTOR_EN, LOW);
    pinMode(Config::MOTOR_EN, OUTPUT);
    digitalWrite(Config::MOTOR_A, LOW);
    digitalWrite(Config::MOTOR_B, LOW);
    pinMode(Config::MOTOR_A, OUTPUT);
    pinMode(Config::MOTOR_B, OUTPUT);
    stopMotor();
}

void commandMotor(MotorDirection requested, uint8_t runPwm, unsigned long now)
{
    if (requested == MotorDirection::Stopped) { stopMotor(); return; }
    if (runPwm < Config::MOTOR_MIN_PWM) runPwm = Config::MOTOR_MIN_PWM;

    if (direction != requested || direction == MotorDirection::Stopped)
    {
        analogWrite(Config::MOTOR_EN, 0);
        applyDirection(requested);
        direction = requested;
        commandStart = now;
        lastRamp = now;
        appliedPwm = Config::MOTOR_START_PWM;
    }
    requestedPwm = runPwm;
    analogWrite(Config::MOTOR_EN, appliedPwm);
}

void updateMotor(unsigned long now)
{
    if (direction == MotorDirection::Stopped) return;
    if (now - commandStart > Config::MOTOR_COMMAND_TIMEOUT_MS)
    {
        stopMotor();
        return;
    }
    if (now - commandStart < Config::MOTOR_START_BOOST_MS) return;
    if (now - lastRamp < Config::MOTOR_RAMP_INTERVAL_MS) return;
    lastRamp = now;
    if (appliedPwm > requestedPwm)
    {
        const uint8_t difference = appliedPwm - requestedPwm;
        appliedPwm -= difference > Config::MOTOR_RAMP_STEP
            ? Config::MOTOR_RAMP_STEP : difference;
    }
    else if (appliedPwm < requestedPwm)
    {
        const uint8_t difference = requestedPwm - appliedPwm;
        appliedPwm += difference > Config::MOTOR_RAMP_STEP
            ? Config::MOTOR_RAMP_STEP : difference;
    }
    analogWrite(Config::MOTOR_EN, appliedPwm);
}

void finishMotorStartBoost()
{
    if (direction == MotorDirection::Stopped) return;
    appliedPwm = requestedPwm;
    analogWrite(Config::MOTOR_EN, appliedPwm);
}

void stopMotor()
{
    analogWrite(Config::MOTOR_EN, 0);
    applyDirection(MotorDirection::Stopped);
    direction = MotorDirection::Stopped;
    requestedPwm = 0;
    appliedPwm = 0;
}

bool isMotorRunning() { return direction != MotorDirection::Stopped; }
MotorDirection getMotorDirection() { return direction; }
uint8_t getAppliedPwm() { return appliedPwm; }
unsigned long getMotorCommandStartTime() { return commandStart; }
