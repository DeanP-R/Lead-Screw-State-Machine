#include "comms.hpp"
#include "config.hpp"

namespace
{
    uint8_t receiveBuffer[Protocol::PacketLength] = {};
    size_t receiveLength = 0;
}

void initialiseComms() { Serial.begin(Config::SERIAL_BAUDRATE); }

PacketStatus receivePacket(Packet& packet, uint8_t& rejectedSequence)
{
    while (Serial.available() > 0)
    {
        const uint8_t byte = static_cast<uint8_t>(Serial.read());
        if (receiveLength == 0 && byte != Protocol::StartByte) continue;
        receiveBuffer[receiveLength++] = byte;
        if (receiveLength < Protocol::PacketLength) continue;

        rejectedSequence = receiveBuffer[2];
        const Protocol::DecodeStatus status = Protocol::decode(receiveBuffer, packet);
        if (status == Protocol::DecodeStatus::Valid)
        {
            receiveLength = 0;
            return PacketStatus::Valid;
        }

        size_t nextStart = 1;
        while (nextStart < Protocol::PacketLength &&
               receiveBuffer[nextStart] != Protocol::StartByte) ++nextStart;
        receiveLength = Protocol::PacketLength - nextStart;
        for (size_t index = 0; index < receiveLength; ++index)
            receiveBuffer[index] = receiveBuffer[nextStart + index];
        if (status == Protocol::DecodeStatus::InvalidCrc) return PacketStatus::InvalidCrc;
    }
    return PacketStatus::NoPacket;
}

bool sendPacket(Response response, uint8_t sequence, int32_t value, uint8_t flags)
{
    const Packet packet {static_cast<uint8_t>(response), sequence, value, flags};
    uint8_t frame[Protocol::PacketLength] = {};
    Protocol::encode(packet, frame);
    return Serial.write(frame, sizeof(frame)) == sizeof(frame);
}

void sendAck(uint8_t sequence) { sendPacket(Response::Ack, sequence); }
void sendNack(uint8_t sequence, NackReason reason)
{
    sendPacket(Response::Nack, sequence, 0, static_cast<uint8_t>(reason));
}
