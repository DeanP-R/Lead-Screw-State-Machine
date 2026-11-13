#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Protocol
{
    constexpr size_t PacketLength = 10;
    constexpr uint8_t StartByte = 0xAA;
    constexpr uint8_t EndByte = 0x55;

    struct Packet
    {
        uint8_t command;
        uint8_t sequence;
        int32_t value;
        uint8_t flags;

        Packet() : command(0), sequence(0), value(0), flags(0) {}
        Packet(uint8_t commandValue, uint8_t sequenceValue,
               int32_t packetValue, uint8_t packetFlags)
            : command(commandValue), sequence(sequenceValue),
              value(packetValue), flags(packetFlags) {}
    };

    enum class DecodeStatus : uint8_t { Valid, InvalidStart, InvalidEnd, InvalidCrc };

    uint8_t crc8(const uint8_t* data, size_t length);
    void encode(const Packet& packet, uint8_t frame[PacketLength]);
    DecodeStatus decode(const uint8_t frame[PacketLength], Packet& packet);
}
