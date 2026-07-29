#include "command_handler.hpp"

#include "comms.hpp"
#include "encoder.hpp"
#include "state_machine.hpp"

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
            if (requestHoming(packet.sequence))
            {
                sendAck(packet.sequence);
            }
            else
            {
                sendNack(
                    packet.sequence,
                    NackReason::Busy
                );
            }

            break;
        }

        case Command::MoveAbsolute:
        {
            if (
                requestMoveAbsolute(
                    packet.sequence,
                    packet.value
                )
            )
            {
                sendAck(packet.sequence);
            }
            else
            {
                sendNack(
                    packet.sequence,
                    NackReason::Busy
                );
            }

            break;
        }

        case Command::MoveRelative:
        {
            if (
                requestMoveRelative(
                    packet.sequence,
                    packet.value
                )
            )
            {
                sendAck(packet.sequence);
            }
            else
            {
                sendNack(
                    packet.sequence,
                    NackReason::Busy
                );
            }

            break;
        }

        case Command::StartTracking:
        {
            if (requestStartTracking(packet.sequence))
            {
                sendAck(packet.sequence);
            }
            else
            {
                sendNack(
                    packet.sequence,
                    NackReason::Busy
                );
            }

            break;
        }

        case Command::StopTracking:
        {
            if (requestStopTracking(packet.sequence))
            {
                sendAck(packet.sequence);
            }
            else
            {
                sendNack(
                    packet.sequence,
                    NackReason::InvalidState
                );
            }

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
            sendStatus(
                packet.sequence,
                getEncoderPosition(),
                static_cast<uint8_t>(
                    getLifterState()
                )
            );

            break;
        }

        case Command::ClearFault:
        {
            clearFault();
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