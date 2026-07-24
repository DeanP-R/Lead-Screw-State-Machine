// =====================================================================================================
// DISTANCE SENSOR
// =====================================================================================================
#include <Arduino.h>

#include "config.hpp"
#include "vl6180x_driver.hpp"

Adafruit_VL6180X vl = Adafruit_VL6180X();

bool readDistance(uint8_t &distanceMm){
  uint8_t measuredRange = vl.readRange();
  uint8_t rangeStatus = vl.readRangeStatus();

  if (rangeStatus != VL6180X_ERROR_NONE)
  {
    return false;
  }

  distanceMm = measuredRange;
  return true;
}

bool initialiseDistanceSensor(){
  if (!vl.begin()){
    Serial.println("FAULT: VL6180X not detected");
    
    return false;
  }
  return true;
}
