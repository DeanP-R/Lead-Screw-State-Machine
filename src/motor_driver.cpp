#include <Arduino.h>

#include "config.hpp"
#include "motor_driver.hpp"

void driveMotorCW(uint8_t pwm){ // this goes down
  digitalWrite(Config::MOTOR_A, HIGH);
  digitalWrite(Config::MOTOR_B, LOW);
  analogWrite(Config::MOTOR_EN, pwm);
}


void driveMotorCCW(uint8_t pwm){ // this goes up 
  digitalWrite(Config::MOTOR_A, LOW);
  digitalWrite(Config::MOTOR_B, HIGH);
  analogWrite(Config::MOTOR_EN, pwm);
}
void initialiseMotor()
{
    pinMode(Config::MOTOR_EN, OUTPUT);
    pinMode(Config::MOTOR_A, OUTPUT);
    pinMode(Config::MOTOR_B, OUTPUT);

    stopMotor();
}

void stopMotor(){
  analogWrite(Config::MOTOR_EN, 0);
  digitalWrite(Config::MOTOR_A, LOW);
  digitalWrite(Config::MOTOR_B, LOW);
}
void pulseMotorCW(){
  driveMotorCW(Config::MOTOR_PWM);
  delay(Config::MOTOR_PULSE_MS);
  stopMotor();
}

void pulseMotorCCW(){
  driveMotorCCW(Config::MOTOR_PWM);
  delay(Config::MOTOR_PULSE_MS);
  stopMotor();
}



