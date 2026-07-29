#include <Arduino.h>

#include "config.hpp"
#include "encoder.hpp"
#include "homing.hpp"
#include "motor_driver.hpp"

namespace
{
    bool homed = false;

    bool validConfiguration()
    {
        if (
            Config::HOME_UPWARD_ENCODER_DIRECTION != 1 &&
            Config::HOME_UPWARD_ENCODER_DIRECTION != -1
        )
        {
            return false;
        }

        if (Config::HOME_INITIAL_MOVEMENT_COUNTS <= 0)
        {
            return false;
        }

        if (Config::HOME_MIN_MOVEMENT_COUNTS <= 0)
        {
            return false;
        }

        if (Config::HOME_BACKOFF_COUNTS <= 0)
        {
            return false;
        }

        if (Config::HOME_REVERSE_MOVEMENT_LIMIT <= 0)
        {
            return false;
        }

        if (Config::HOME_INITIAL_STALL_TIME_MS == 0)
        {
            return false;
        }

        if (Config::HOME_STALL_TIME_MS == 0)
        {
            return false;
        }

        if (Config::HOME_TIMEOUT_MS == 0)
        {
            return false;
        }

        if (Config::HOME_BACKOFF_TIMEOUT_MS == 0)
        {
            return false;
        }

        return true;
    }

    bool movedInDirection(
        long currentPosition,
        long referencePosition,
        int8_t direction,
        long minimumMovement
    )
    {
        const long movement =
            currentPosition - referencePosition;

        if (direction > 0)
        {
            return movement >= minimumMovement;
        }

        return movement <= -minimumMovement;
    }

    bool movedAgainstDirection(
        long currentPosition,
        long referencePosition,
        int8_t expectedDirection,
        long reverseMovementLimit
    )
    {
        const long movement =
            currentPosition - referencePosition;

        if (expectedDirection > 0)
        {
            return movement <= -reverseMovementLimit;
        }

        return movement >= reverseMovementLimit;
    }

    bool targetReached(
        long currentPosition,
        long targetPosition,
        int8_t direction
    )
    {
        if (direction > 0)
        {
            return currentPosition >= targetPosition;
        }

        return currentPosition <= targetPosition;
    }
}

bool isHomed()
{
    return homed;
}

void clearHomedState()
{
    homed = false;
}

HomingResult homing()
{
    /*
     * Homing sequence:
     *
     * 1. Drive upwards towards the mechanical stop.
     * 2. If substantial encoder movement occurs, monitor progress until
     *    the mechanism stalls against the stop.
     * 3. If no substantial movement occurs shortly after starting,
     *    assume the mechanism was already at the upper stop.
     * 4. Back away by HOME_BACKOFF_COUNTS.
     * 5. Set the backed-off position as encoder zero.
     *
     * The backoff phase verifies that the motor and encoder are operating.
     */

    homed = false;

    stopMotor();
    // return HomingResult::InvalidConfiguration;

    if (!validConfiguration())
    {
        return HomingResult::InvalidConfiguration;
    }

    const int8_t upwardEncoderDirection =
        Config::HOME_UPWARD_ENCODER_DIRECTION;

    const int8_t backoffEncoderDirection =
        -upwardEncoderDirection;

    // ============================================================
    // PHASE 1: SEEK UPPER MECHANICAL STOP
    // ============================================================

    const unsigned long seekStartTime = millis();

    unsigned long lastProgressTime =
        seekStartTime;

    const long seekStartPosition =
        getEncoderPosition();

    long lastProgressPosition =
        seekStartPosition;

    bool movementObserved = false;

    driveMotorCCW(Config::MOTOR_PWM_SLOW);

    while (true)
    {
        const unsigned long currentTime =
            millis();

        const long currentPosition =
            getEncoderPosition();

        /*
         * Before genuine movement is recognised, compare against the
         * original starting position.
         *
         * This prevents small encoder fluctuations at the mechanical
         * stop from immediately being treated as real travel.
         */
        if (!movementObserved)
        {
            if (movedAgainstDirection(
                    currentPosition,
                    seekStartPosition,
                    upwardEncoderDirection,
                    Config::HOME_REVERSE_MOVEMENT_LIMIT
                ))
            {
                stopMotor();

                return HomingResult::SeekReverseMovement;
            }

            if (movedInDirection(
                    currentPosition,
                    seekStartPosition,
                    upwardEncoderDirection,
                    Config::HOME_INITIAL_MOVEMENT_COUNTS
                ))
            {
                movementObserved = true;

                lastProgressPosition =
                    currentPosition;

                lastProgressTime =
                    currentTime;
            }
        }
        else
        {
            /*
             * Check for movement in the wrong direction before updating
             * the progress reference.
             */
            if (movedAgainstDirection(
                    currentPosition,
                    lastProgressPosition,
                    upwardEncoderDirection,
                    Config::HOME_REVERSE_MOVEMENT_LIMIT
                ))
            {
                stopMotor();

                return HomingResult::SeekReverseMovement;
            }

            /*
             * Record meaningful continued movement towards the stop.
             */
            if (movedInDirection(
                    currentPosition,
                    lastProgressPosition,
                    upwardEncoderDirection,
                    Config::HOME_MIN_MOVEMENT_COUNTS
                ))
            {
                lastProgressPosition =
                    currentPosition;

                lastProgressTime =
                    currentTime;
            }
        }

        /*
         * The mechanism travelled and has now stopped making meaningful
         * progress. Treat this as reaching the upper mechanical stop.
         */
        if (
            movementObserved &&
            currentTime - lastProgressTime >=
                Config::HOME_STALL_TIME_MS
        )
        {
            stopMotor();
            break;
        }

        /*
         * No substantial movement occurred after commanding upwards.
         *
         * Assume the mechanism started against the upper mechanical stop.
         * The backoff phase must still demonstrate valid motor and encoder
         * movement before homing succeeds.
         */
        if (
            !movementObserved &&
            currentTime - seekStartTime >=
                Config::HOME_INITIAL_STALL_TIME_MS
        )
        {
            stopMotor();
            break;
        }

        /*
         * Overall seek timeout only applies after genuine movement has
         * begun.
         */
        if (
            movementObserved &&
            currentTime - seekStartTime >=
                Config::HOME_TIMEOUT_MS
        )
        {
            stopMotor();

            return HomingResult::SeekTimeout;
        }
    }

    delay(Config::HOME_SETTLE_TIME_MS);

    // ============================================================
    // PHASE 2: BACK OFF FROM UPPER MECHANICAL STOP
    // ============================================================

    const long backoffStartPosition =
        getEncoderPosition();

    const long backoffTargetPosition =
        backoffStartPosition +
        (
            static_cast<long>(backoffEncoderDirection) *
            Config::HOME_BACKOFF_COUNTS
        );

    const unsigned long backoffStartTime =
        millis();

    unsigned long lastBackoffProgressTime =
        backoffStartTime;

    long lastBackoffProgressPosition =
        backoffStartPosition;

    bool backoffMovementObserved = false;

    driveMotorCW(Config::MOTOR_PWM_SLOW);

    while (true)
    {
        const unsigned long currentTime =
            millis();

        const long currentPosition =
            getEncoderPosition();

        /*
         * Check for movement opposite to the expected backoff direction
         * before updating the progress reference.
         */
        if (movedAgainstDirection(
                currentPosition,
                lastBackoffProgressPosition,
                backoffEncoderDirection,
                Config::HOME_REVERSE_MOVEMENT_LIMIT
            ))
        {
            stopMotor();

            return HomingResult::BackoffReverseMovement;
        }

        /*
         * Record meaningful backoff movement.
         */
        if (movedInDirection(
                currentPosition,
                lastBackoffProgressPosition,
                backoffEncoderDirection,
                Config::HOME_MIN_MOVEMENT_COUNTS
            ))
        {
            lastBackoffProgressPosition =
                currentPosition;

            lastBackoffProgressTime =
                currentTime;

            backoffMovementObserved = true;
        }

        /*
         * Required backoff distance reached.
         */
        if (targetReached(
                currentPosition,
                backoffTargetPosition,
                backoffEncoderDirection
            ))
        {
            stopMotor();
            break;
        }

        /*
         * Backoff began but then stopped before reaching the target.
         */
        if (
            backoffMovementObserved &&
            currentTime - lastBackoffProgressTime >=
                Config::HOME_STALL_TIME_MS
        )
        {
            stopMotor();

            return HomingResult::BackoffStall;
        }

        /*
         * No meaningful backoff movement occurred.
         *
         * This catches a disconnected motor, failed encoder, incorrect
         * motor direction or a mechanism jammed in both directions.
         */
        if (
            !backoffMovementObserved &&
            currentTime - backoffStartTime >=
                Config::HOME_BACKOFF_TIMEOUT_MS
        )
        {
            stopMotor();

            return HomingResult::BackoffNoMovement;
        }

        /*
         * Movement occurred, but the requested backoff distance was not
         * reached within the permitted time.
         */
        if (
            backoffMovementObserved &&
            currentTime - backoffStartTime >=
                Config::HOME_BACKOFF_TIMEOUT_MS
        )
        {
            stopMotor();

            return HomingResult::BackoffTimeout;
        }
    }

    delay(Config::HOME_SETTLE_TIME_MS);

    // ============================================================
    // PHASE 3: ESTABLISH SOFTWARE ZERO
    // ============================================================

    setEncoderPosition(0);

    if (getEncoderPosition() != 0)
    {
        stopMotor();

        homed = false;

        return HomingResult::ZeroingFailed;
    }

    stopMotor();

    homed = true;

    return HomingResult::Success;
}