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
     *
     * Both the seek and backoff phases apply a short grace period at
     * start-of-move before enforcing the reverse-movement check. This
     * tolerates normal gear backlash / stop-rebound noise right as the
     * motor starts driving, without masking genuine reverse-direction
     * faults (wrong wiring, motor driving away from the stop, etc.),
     * which will still be caught once the grace period elapses.
     */

    homed = false;

    stopMotor();

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
    const long seekStartPosition = getEncoderPosition();

    bool movementObserved = false;

    // Used only for the "genuine stall" check once movement has begun.
    unsigned long lastWindowTime = seekStartTime;
    long lastWindowPosition = seekStartPosition;

    driveMotorCCW(Config::MOTOR_PWM_SLOW);

    while (true)
    {
        const unsigned long currentTime = millis();
        const long currentPosition = getEncoderPosition();

        // --- Case A: reverse movement is a fault, once past the start-of-move grace period ---
        const bool pastReverseCheckGrace =
            (currentTime - seekStartTime) >= Config::HOME_REVERSE_CHECK_GRACE_MS;

        if (pastReverseCheckGrace)
        {
            const long referenceForReverseCheck =
                movementObserved ? lastWindowPosition : seekStartPosition;

            if (movedAgainstDirection(
                    currentPosition,
                    referenceForReverseCheck,
                    upwardEncoderDirection,
                    Config::HOME_REVERSE_MOVEMENT_LIMIT
                ))
            {
                stopMotor();
                return HomingResult::SeekReverseMovement;
            }
        }

        // --- Case B: detect that real movement has begun ---
        if (!movementObserved &&
            movedInDirection(
                currentPosition,
                seekStartPosition,
                upwardEncoderDirection,
                Config::HOME_INITIAL_MOVEMENT_COUNTS
            ))
        {
            movementObserved = true;
            lastWindowPosition = currentPosition;
            lastWindowTime = currentTime;
        }

        // --- Case C: once moving, check for a genuine stall on a fixed time window ---
        if (movementObserved &&
            currentTime - lastWindowTime >= Config::HOME_STALL_TIME_MS)
        {
            const bool madeEnoughProgress = movedInDirection(
                currentPosition,
                lastWindowPosition,
                upwardEncoderDirection,
                Config::HOME_MIN_MOVEMENT_COUNTS
            );

            if (!madeEnoughProgress)
            {
                stopMotor();
                break; // genuine stall against the stop — success path
            }

            // Enough progress this window — slide the window forward and keep going.
            lastWindowPosition = currentPosition;
            lastWindowTime = currentTime;
        }

        // --- Case D: never got moving at all — assume already at the stop ---
        if (!movementObserved &&
            currentTime - seekStartTime >= Config::HOME_INITIAL_STALL_TIME_MS)
        {
            stopMotor();
            break; // treated as already-homed-ish; backoff phase will verify motor/encoder
        }

        // --- Case E: took too long overall once moving ---
        if (movementObserved &&
            currentTime - seekStartTime >= Config::HOME_TIMEOUT_MS)
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
         * Check for movement opposite to the expected backoff direction,
         * once past the start-of-move grace period. Backlash/rebound
         * noise right as the motor reverses off the stop is tolerated;
         * genuine reverse faults are still caught afterwards.
         */
        const bool pastBackoffReverseCheckGrace =
            (currentTime - backoffStartTime) >= Config::HOME_REVERSE_CHECK_GRACE_MS;

        if (pastBackoffReverseCheckGrace &&
            movedAgainstDirection(
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