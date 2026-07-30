#include "state_machine.hpp"

#include <stdlib.h>

#include "config.hpp"
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

    // Set once, on the first tick after entering MovingRelative /
    // MovingAbsolute, then held for the duration of that move.
    bool movementTargetSet = false;
    long movementTargetPosition = 0;
    unsigned long movementStartTime = 0;

    // Tracks the most recent point at which meaningful progress toward
    // the target was observed, to detect a stalled/blocked move.
    long movementProgressPosition = 0;
    unsigned long movementProgressTime = 0;

    // Fault reason codes reported via Response::Fault's flags byte
    // when a commanded move fails. Kept local to this file since
    // callers only ever see them echoed back over the wire.
    enum class MoveFault : uint8_t
    {
        Timeout = 0x01,
        Stall = 0x02
    };

    void resetMovementTracking()
    {
        movementTargetSet = false;
    }
}

void initialiseStateMachine()
{
    stopMotor();
    clearHomedState();

    currentState = LifterState::Idle;
    activeSequence = 0;

    requestedPosition = 0;
    requestedMovement = 0;

    resetMovementTracking();
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
    resetMovementTracking();
    currentState = LifterState::Idle;
}

void clearFault()
{
    if (currentState == LifterState::Fault)
    {
        resetMovementTracking();
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
            const unsigned long currentTime = millis();
            const long currentPosition = getEncoderPosition();

            if (!movementTargetSet)
            {
                movementTargetPosition =
                    currentPosition + requestedMovement;

                movementStartTime = currentTime;
                movementProgressTime = currentTime;
                movementProgressPosition = currentPosition;
                movementTargetSet = true;
            }

            const long remaining =
                movementTargetPosition - currentPosition;

            /*
             * Close enough — treat the move as complete. Chasing an
             * exact count would fight encoder noise and backlash near
             * the target forever.
             */
            if (labs(remaining) <= Config::MOVE_TOLERANCE_COUNTS)
            {
                stopMotor();
                resetMovementTracking();

                sendPacket(
                    Response::MoveComplete,
                    activeSequence,
                    currentPosition,
                    static_cast<uint8_t>(LifterState::Idle)
                );

                currentState = LifterState::Idle;

                break;
            }

            /*
             * Safety cutoff if the target is never reached (mechanism
             * jammed, unreachable target, etc).
             */
            if (currentTime - movementStartTime >= Config::MOVE_TIMEOUT_MS)
            {
                stopMotor();
                resetMovementTracking();

                sendPacket(
                    Response::Fault,
                    activeSequence,
                    currentPosition,
                    static_cast<uint8_t>(MoveFault::Timeout)
                );

                currentState = LifterState::Fault;

                break;
            }

            /*
             * Stall protection: if the mechanism is blocked (e.g. the
             * target is past the physical end of travel), don't keep
             * driving into it for the full timeout — stop as soon as
             * progress toward the target has genuinely stopped.
             */
            if (labs(currentPosition - movementProgressPosition) >=
                Config::MOVE_MIN_PROGRESS_COUNTS)
            {
                movementProgressPosition = currentPosition;
                movementProgressTime = currentTime;
            }
            else if (currentTime - movementProgressTime >=
                     Config::MOVE_STALL_TIME_MS)
            {
                stopMotor();
                resetMovementTracking();

                sendPacket(
                    Response::Fault,
                    activeSequence,
                    currentPosition,
                    static_cast<uint8_t>(MoveFault::Stall)
                );

                currentState = LifterState::Fault;

                break;
            }

            /*
             * Not there yet — nudge one pulse closer. Positive
             * `remaining` means the target is above the current
             * position (encoder increasing = upward, per
             * driveMotorCCW's convention in motor_driver.cpp).
             */
            if (remaining > 0)
            {
                pulseMotorCCW();
            }
            else
            {
                pulseMotorCW();
            }

            break;
        }

        case LifterState::Tracking:
        {
            standoff_tracking();
            break;
        }

        case LifterState::Fault:
        {
            stopMotor();
            break;
        }
    }
}