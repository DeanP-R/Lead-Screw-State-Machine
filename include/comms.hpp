#pragma once

#include <Arduino.h>

struct Packet
{
    uint8_t command;
    uint8_t sequence;
    int32_t value;
    uint8_t flags;
};

enum class PacketStatus : uint8_t
{
    NoPacket,
    InvalidStart,
    InvalidEnd,
    InvalidCrc,
    Valid
};

enum class Command : uint8_t
{
    Home          = 0x01,
    MoveAbsolute  = 0x02,
    MoveRelative  = 0x03,
    StartTracking = 0x04,
    StopTracking  = 0x05,
    Stop          = 0x06,
    GetStatus     = 0x07,
    ClearFault    = 0x08,
    SetStandoff   = 0x09,
    Ping          = 0x10
};

enum class Response : uint8_t
{
    Ack          = 0x80,
    Nack         = 0x81,
    Status       = 0x82,
    HomeComplete = 0x83,
    MoveComplete = 0x84,
    Fault        = 0x85
};

enum class NackReason : uint8_t
{
    BadCrc        = 0x01,
    UnknownCommand = 0x02,
    InvalidState   = 0x03,
    NotHomed       = 0x04,
    OutOfRange     = 0x05,
    Busy           = 0x06
};

void initialiseComms();

uint8_t calculateCrc8(
    const uint8_t* data,
    size_t length
);
void sendStatus(
    uint8_t sequence,
    int32_t encoderPosition,
    uint8_t state
);
PacketStatus receivePacket(Packet& packet);

bool sendPacket(
    Response response,
    uint8_t sequence,
    int32_t value = 0,
    uint8_t flags = 0
);

void sendAck(uint8_t sequence);

void sendNack(
    uint8_t sequence,
    NackReason reason
);



