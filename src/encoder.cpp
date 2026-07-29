#include <Arduino.h>

#include "config.hpp"
#include "motor_driver.hpp"
#include "encoder.hpp"


namespace
{
    volatile long encoderPosition = 0;
}
void initialiseEncoder()
{
    pinMode(Config::ENCODER_A, INPUT);
    pinMode(Config::ENCODER_B, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(Config::ENCODER_A),
        readEncoder,
        CHANGE
    );
}
long getEncoderPosition(){
  noInterrupts();
  long position = encoderPosition;
  interrupts();

  return position;
}
void setEncoderPosition(long position){
  noInterrupts();
  encoderPosition = position;
  interrupts();
}



void readEncoder(){
  if (digitalRead(Config::ENCODER_A) != digitalRead(Config::ENCODER_B))
  {
    encoderPosition--;
  }
  else
  {
    encoderPosition++;
  }
}
