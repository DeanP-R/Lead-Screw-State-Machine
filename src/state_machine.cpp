#include "state_machine.hpp"

#include "comms.hpp"
#include "encoder.hpp"
#include "motor_driver.hpp"
#include "homing.hpp"
#include "tracking.hpp"

namespace
{
    LifterState currentState = LifterState::Idle;

    uint8_t activeSequence = 0;

    int32_t requestedPosition = 0;
    int32_t requestedMovement = 0;

}

void initialiseStateMachine()
{
    stopMotor();
    clearHomedState();

    currentState = LifterState::Idle;
    activeSequence = 0;

    requestedPosition = 0;
    requestedMovement = 0;
}
LifterState getLifterState()
{
    return currentState;
}

bool requestHoming(uint8_t sequence)
{
    if (currentState != LifterState::Idle)
    {
        return false;
    }

    activeSequence = sequence;
    currentState = LifterState::Homing;

    return true;
}

bool requestMoveAbsolute(
    uint8_t sequence,
    int32_t targetPosition
)
{
    if (currentState != LifterState::Idle)
    {
        return false;
    }

    activeSequence = sequence;
    requestedPosition = targetPosition;
    currentState = LifterState::MovingAbsolute;

    return true;
}

bool requestMoveRelative(
    uint8_t sequence,
    int32_t movement
)
{
    if (currentState != LifterState::Idle)
    {
        return false;
    }

    activeSequence = sequence;
    requestedMovement = movement;
    currentState = LifterState::MovingRelative;

    return true;
}

bool requestStartTracking(uint8_t sequence)
{
    if (currentState != LifterState::Idle)
    {
        return false;
    }

    activeSequence = sequence;
    currentState = LifterState::Tracking;

    return true;
}

bool requestStopTracking(uint8_t sequence)
{
    if (currentState != LifterState::Tracking)
    {
        return false;
    }

    activeSequence = sequence;

    stopMotor();

    currentState = LifterState::Idle;

    return true;
}

void requestStop()
{
    stopMotor();
    currentState = LifterState::Idle;
}

void clearFault()
{
    if (currentState == LifterState::Fault)
    {
        currentState = LifterState::Idle;
    }
}

void updateStateMachine()
{
    switch (currentState)
    {
        case LifterState::Idle:
        {
            stopMotor();
            break;
        }
        case LifterState::Homing:
        {
            const HomingResult result = homing();

            if (result == HomingResult::Success)
            {
                currentState = LifterState::Idle;

                sendPacket(
                    Response::HomeComplete,
                    activeSequence,
                    getEncoderPosition(),
                    static_cast<uint8_t>(currentState)
                );
            }
            else
            {
                currentState = LifterState::Fault;

                sendPacket(
                    Response::Fault,
                    activeSequence,
                    getEncoderPosition(),
                    static_cast<uint8_t>(result)
                );
            }

            break;
        }

        case LifterState::MovingAbsolute:
        {
            /*
            #TODO:

            const MovementStatus status =
                updateAbsoluteMovement(requestedPosition);

            if (status == MovementStatus::Complete)
            {
                sendPacket(
                    Response::MoveComplete,
                    activeSequence,
                    getEncoderPosition(),
                    0
                );

                currentState = LifterState::Idle;
            }
            */

            break;
        }

        case LifterState::MovingRelative:
        {
            /*
            #TODO:

            const MovementStatus status =
                updateRelativeMovement(requestedMovement);
            */

            break;
        }

        case LifterState::Tracking:
        {
            standoff_tracking();
        }

        case LifterState::Fault:
        {
            stopMotor();
            break;
        }
    }
}