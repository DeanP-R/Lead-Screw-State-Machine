#include <Arduino.h>

#include "config.hpp"
#include "comms.hpp"

void initialiseComms()
{
    Serial.begin(Config::SERIAL_BAUDRATE);
}

uint8_t calculateCrc8(const uint8_t* data, size_t length)
{
    uint8_t crc = Config::CRC_INITIAL;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = static_cast<uint8_t>(
                    (crc << 1U) ^ Config::CRC_POLYNOMIAL
                );
            }
            else
            {
                crc = static_cast<uint8_t>(crc << 1U);
            }
        }
    }

    return crc;
}
void sendStatus(
    uint8_t sequence,
    int32_t encoderPosition,
    uint8_t state
)
{
    sendPacket(
        Response::Status,
        sequence,
        encoderPosition,
        state
    );
}
bool sendPacket(
    Response response,
    uint8_t sequence,
    int32_t value,
    uint8_t flags
)
{
    uint8_t msg[Config::PACKET_LENGTH] = {};

    msg[Config::START_INDEX] = Config::START_BYTE;
    msg[Config::COMMAND_INDEX] = static_cast<uint8_t>(response);
    msg[Config::SEQUENCE_INDEX] = sequence;

    const uint32_t rawValue = static_cast<uint32_t>(value);

    msg[Config::VALUE_INDEX] =
        static_cast<uint8_t>(rawValue & 0xFFU);

    msg[Config::VALUE_INDEX + 1] =
        static_cast<uint8_t>((rawValue >> 8U) & 0xFFU);

    msg[Config::VALUE_INDEX + 2] =
        static_cast<uint8_t>((rawValue >> 16U) & 0xFFU);

    msg[Config::VALUE_INDEX + 3] =
        static_cast<uint8_t>((rawValue >> 24U) & 0xFFU);

    msg[Config::FLAGS_INDEX] = flags;

    msg[Config::CRC_INDEX] = calculateCrc8(
        &msg[Config::COMMAND_INDEX],
        Config::CRC_INDEX - Config::COMMAND_INDEX
    );

    msg[Config::END_INDEX] = Config::END_BYTE;

    const size_t bytesWritten = Serial.write(
        msg,
        Config::PACKET_LENGTH
    );

    return bytesWritten == Config::PACKET_LENGTH;
}

void sendAck(uint8_t sequence)
{
    sendPacket(
        Response::Ack,
        sequence,
        0,
        0
    );
}

void sendNack(
    uint8_t sequence,
    NackReason reason
)
{
    sendPacket(
        Response::Nack,
        sequence,
        0,
        static_cast<uint8_t>(reason)
    );
}

PacketStatus receivePacket(Packet& packet)
{
    if (Serial.available() < Config::PACKET_LENGTH)
    {
        return PacketStatus::NoPacket;
    }

    uint8_t msg[Config::PACKET_LENGTH];

    const size_t bytesRead = Serial.readBytes(
        msg,
        Config::PACKET_LENGTH
    );

    if (bytesRead != Config::PACKET_LENGTH)
    {
        return PacketStatus::NoPacket;
    }

    if (msg[Config::START_INDEX] != Config::START_BYTE)
    {
        return PacketStatus::InvalidStart;
    }

    if (msg[Config::END_INDEX] != Config::END_BYTE)
    {
        return PacketStatus::InvalidEnd;
    }

    const uint8_t calculatedCrc = calculateCrc8(
        &msg[Config::COMMAND_INDEX],
        Config::CRC_INDEX - Config::COMMAND_INDEX
    );

    if (msg[Config::CRC_INDEX] != calculatedCrc)
    {
        return PacketStatus::InvalidCrc;
    }

    packet.command = msg[Config::COMMAND_INDEX];
    packet.sequence = msg[Config::SEQUENCE_INDEX];
    packet.flags = msg[Config::FLAGS_INDEX];

    const uint32_t rawValue =
        static_cast<uint32_t>(
            msg[Config::VALUE_INDEX]
        )
        |
        (
            static_cast<uint32_t>(
                msg[Config::VALUE_INDEX + 1]
            ) << 8U
        )
        |
        (
            static_cast<uint32_t>(
                msg[Config::VALUE_INDEX + 2]
            ) << 16U
        )
        |
        (
            static_cast<uint32_t>(
                msg[Config::VALUE_INDEX + 3]
            ) << 24U
        );

    packet.value = static_cast<int32_t>(rawValue);

    return PacketStatus::Valid;
}