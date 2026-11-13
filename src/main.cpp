// Safety-oriented ACFM lifter replacement for Arduino Nano ATmega328P.
// Automatic movement is compile-time inhibited until physical commissioning.
#include "comms.hpp"
#include "command_handler.hpp"
#include "state_machine.hpp"
#include "motor_driver.hpp"
#include "encoder.hpp"
#include "distance_sensor.hpp"

void setup()
{
    // Disable the bridge before starting Serial/Wire or any state logic.
    initialiseMotor();
    initialiseEncoder();
    initialiseComms();
    const bool sensorInitialised = initialiseDistanceSensor();
    initialiseStateMachine(sensorInitialised, millis());
}

void loop()
{
    Packet packet;
    uint8_t rejectedSequence = 0;

    const PacketStatus status = receivePacket(packet, rejectedSequence);

    if (status == PacketStatus::Valid)
    {
        handlePacket(packet);
    }
    else if (status == PacketStatus::InvalidCrc)
    {
        sendNack(
            rejectedSequence,
            NackReason::BadCrc
        );
    }

    updateStateMachine(millis());
}
