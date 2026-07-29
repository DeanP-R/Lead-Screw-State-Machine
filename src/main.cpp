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
#include "comms.hpp"
#include "command_handler.hpp"
#include "state_machine.hpp"
#include "motor_driver.hpp"
#include "encoder.hpp"
#include "vl6180x_driver.hpp"

void setup()
{
    initialiseComms();
    initialiseStateMachine();

    initialiseMotor();
    initialiseEncoder();
    initialiseDistanceSensor();
}

void loop()
{
    Packet packet {};

    const PacketStatus status = receivePacket(packet);

    if (status == PacketStatus::Valid)
    {
        handlePacket(packet);
    }
    else if (status == PacketStatus::InvalidCrc)
    {
        sendNack(
            packet.sequence,
            NackReason::BadCrc
        );
    }

    updateStateMachine();
}