/********************************************************************************************************
* Filename      : screw_lifter_driver.ino
* Version       : 0.4.0
* Description   : Moves the screw lifter to maintain a 10 mm measured distance using a VL6180X.
* Author        : Dean Rowlett
* Target        : Arduino Nano ATmega328P
* IDE           : Arduino IDE
* Last Updated  : 24 July 2026
*********************************************************************************************************
Algorithm Logic
  Initialise 
    - Configure pins
    - Configure Sensors
    - Setup VL6180X
    - HOMING motor (Maybe?)
  Forever
    - Read TOF sensor
      - Is reading valid?
        - Is stand-off larger thgan target distance + tolerance?
          - Move Probem downward (towards surface)
          - Stop Motor
          - Return
        - Is stand-off less than target distance - tolerance?
          - Move probe upward (away from surface)
          - Stop motor
          - Return
        - Stop Motor (Hold Position)
        - Return
      - Report Error
      - Stop motor
      - Return
Pseudo-code
  BEGIN
    Initialise motor driver
    Initialise encoder
    Initialise distance sensor
  LOOP forever
      Read distance sensor
      IF sensor reading is invalid THEN
          Stop motor
          Report sensor fault
          Continue
      ENDIF
      IF measured_distance > target_distance + tolerance THEN
          Move motor towards surface (CCW)
          Wait short period
          Stop motor
      ELSE IF measured_distance < target_distance - tolerance THEN
          Move motor away from surface
          Wait short period
          Stop motor
      ELSE
          Stop motor
      ENDIF
  END LOOP
  END
********************************************************************************************************/
#include "config.hpp"
#include "motor_driver.hpp"
#include "vl6180x_driver.hpp"
#include "tracking.hpp"
#include "encoder.hpp"
#include "homing.hpp"

constexpr uint8_t LED_PIN = 13;


void loop(){
  // standoff_tracking();
  byte msg[10];

  if (Serial.readBytes(msg, 10) == 10)
  {
      if (msg[0] != 0xAA) //Wrong start byte
      {
          return;
      }

      if (msg[9] != 0x55) // Wrong end byte
      {
          return;
      }

      // Temporary CRC check
      if (msg[8] != 0xD5) //Wrong CRC 
      {
          return;
      }

      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(115200);
    Wire.begin();

    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);

    // initialiseMotor();
    // initialiseEncoder();

    // if (!initialiseDistanceSensor())
    // {
    //     stopMotor();

    //     while (true)
    //     {
    //         delay(1000);
    //     }
    // }

    // Serial.println("Sensor init OK");

    // Serial.print("HOMING: ");
    // Serial.println(homing());

    // Serial.println("READY");
}