#include <Arduino.h>
#include "config.hpp"
#include "motor_driver.hpp"
#include "vl6180x_driver.hpp"

void standoff_tracking(){
  uint8_t distanceMm = 0;
  if (!readDistance(distanceMm)){
    return;
  }
  else{

    if((distanceMm) > (Config::TARGET_MM + Config::TOLERANCE_MM))
    {
      pulseMotorCW();
      stopMotor();
      return;
    }
    else if((distanceMm) < (Config::TARGET_MM - Config::TOLERANCE_MM))
    {
      pulseMotorCCW();
      stopMotor();
      return;
    }
    else{
      stopMotor();
      return;
    }
    
  }
}
