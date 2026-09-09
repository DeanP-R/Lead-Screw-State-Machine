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
    // The encoder outputs are open-collector on the deployed hardware.
    // Without pull-ups the inputs float and can produce only a few net
    // counts during an otherwise real motor movement.
    pinMode(Config::ENCODER_A, INPUT_PULLUP);
    pinMode(Config::ENCODER_B, INPUT_PULLUP);
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
