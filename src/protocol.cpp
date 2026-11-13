#include "protocol.hpp"

namespace Protocol
{
    uint8_t crc8(const uint8_t* data, size_t length)
    {
        uint8_t crc = 0;
        for (size_t index = 0; index < length; ++index)
        {
            crc ^= data[index];
            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x80U)
                    ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
                    : static_cast<uint8_t>(crc << 1U);
            }
        }
        return crc;
    }

    void encode(const Packet& packet, uint8_t frame[PacketLength])
    {
        frame[0] = StartByte;
        frame[1] = packet.command;
        frame[2] = packet.sequence;
        const uint32_t value = static_cast<uint32_t>(packet.value);
        frame[3] = static_cast<uint8_t>(value);
        frame[4] = static_cast<uint8_t>(value >> 8U);
        frame[5] = static_cast<uint8_t>(value >> 16U);
        frame[6] = static_cast<uint8_t>(value >> 24U);
        frame[7] = packet.flags;
        frame[8] = crc8(&frame[1], 7);
        frame[9] = EndByte;
    }

    DecodeStatus decode(const uint8_t frame[PacketLength], Packet& packet)
    {
        if (frame[0] != StartByte) return DecodeStatus::InvalidStart;
        if (frame[9] != EndByte) return DecodeStatus::InvalidEnd;
        if (frame[8] != crc8(&frame[1], 7)) return DecodeStatus::InvalidCrc;
        packet.command = frame[1];
        packet.sequence = frame[2];
        const uint32_t value = static_cast<uint32_t>(frame[3]) |
            (static_cast<uint32_t>(frame[4]) << 8U) |
            (static_cast<uint32_t>(frame[5]) << 16U) |
            (static_cast<uint32_t>(frame[6]) << 24U);
        packet.value = static_cast<int32_t>(value);
        packet.flags = frame[7];
        return DecodeStatus::Valid;
    }
}
