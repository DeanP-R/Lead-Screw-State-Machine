#include "command_handler.hpp"

#include "comms.hpp"
#include "encoder.hpp"
#include "state_machine.hpp"
#include "tracking.hpp"

namespace
{
    NackReason nackFor(RequestResult result)
    {
        switch (result)
        {
            case RequestResult::Busy: return NackReason::Busy;
            case RequestResult::NotHomed: return NackReason::NotHomed;
            case RequestResult::OutOfRange: return NackReason::OutOfRange;
            case RequestResult::NotCommissioned: return NackReason::NotCommissioned;
            case RequestResult::SensorInvalid: return NackReason::SensorInvalid;
            default: return NackReason::InvalidState;
        }
    }

    void acknowledgeResult(uint8_t sequence, RequestResult result)
    {
        if (result == RequestResult::Accepted) sendAck(sequence);
        else sendNack(sequence, nackFor(result));
    }
}

void handlePacket(const Packet& packet)
{
    const Command command =
        static_cast<Command>(packet.command);

    switch (command)
    {
        case Command::Ping:
        {
            sendAck(packet.sequence);
            break;
        }

        case Command::Home:
        {
            acknowledgeResult(packet.sequence,
                              requestHoming(packet.sequence, millis()));

            break;
        }

        case Command::MoveAbsolute:
        {
            acknowledgeResult(packet.sequence, requestMoveAbsolute(
                packet.sequence, packet.value, millis()));

            break;
        }

        case Command::MoveRelative:
        {
            acknowledgeResult(packet.sequence, requestMoveRelative(
                packet.sequence, packet.value, millis()));

            break;
        }

        case Command::StartTracking:
        {
            acknowledgeResult(packet.sequence, requestStartTracking(
                packet.sequence, millis()));

            break;
        }

        case Command::StopTracking:
        {
            acknowledgeResult(packet.sequence, requestStopTracking());

            break;
        }

        case Command::Stop:
        {
            requestStop();
            sendAck(packet.sequence);
            break;
        }

        case Command::GetStatus:
        {
            sendPacket(Response::Status, packet.sequence, getEncoderPosition(),
                       static_cast<uint8_t>(getLifterState()));

            break;
        }

        case Command::ClearFault:
        {
            if (clearFault()) sendAck(packet.sequence);
            else sendNack(packet.sequence, NackReason::InvalidState);
            break;
        }

        case Command::SetStandoff:
        {
            if (packet.value > 0 && packet.value <= 2000 &&
                setTrackingStandoff(static_cast<uint16_t>(packet.value)))
                sendAck(packet.sequence);
            else sendNack(packet.sequence, NackReason::OutOfRange);
            break;
        }

        case Command::EnableTelemetry:
        {
            setTelemetryEnabled(true);
            sendAck(packet.sequence);
            break;
        }

        case Command::DisableTelemetry:
        {
            setTelemetryEnabled(false);
            sendAck(packet.sequence);
            break;
        }

        default:
        {
            sendNack(
                packet.sequence,
                NackReason::UnknownCommand
            );

            break;
        }
    }
}
