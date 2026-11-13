#pragma once

#include <Arduino.h>
#include "protocol.hpp"

using Packet = Protocol::Packet;

enum class PacketStatus : uint8_t { NoPacket, InvalidCrc, Valid };

enum class Command : uint8_t
{
    Home = 0x01, MoveAbsolute = 0x02, MoveRelative = 0x03,
    StartTracking = 0x04, StopTracking = 0x05, Stop = 0x06,
    GetStatus = 0x07, ClearFault = 0x08, SetStandoff = 0x09,
    EnableTelemetry = 0x0A, DisableTelemetry = 0x0B, Ping = 0x10
};

enum class Response : uint8_t
{
    Ack = 0x80, Nack = 0x81, Status = 0x82, HomeComplete = 0x83,
    MoveComplete = 0x84, Fault = 0x85, Distance = 0x86,
    Encoder = 0x87, SensorDiagnostic = 0x88
};

enum class NackReason : uint8_t
{
    BadCrc = 0x01, UnknownCommand = 0x02, InvalidState = 0x03,
    NotHomed = 0x04, OutOfRange = 0x05, Busy = 0x06,
    NotCommissioned = 0x07, SensorInvalid = 0x08
};

void initialiseComms();
PacketStatus receivePacket(Packet& packet, uint8_t& rejectedSequence);
bool sendPacket(Response response, uint8_t sequence, int32_t value = 0, uint8_t flags = 0);
void sendAck(uint8_t sequence);
void sendNack(uint8_t sequence, NackReason reason);
