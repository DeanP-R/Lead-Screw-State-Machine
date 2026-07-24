#include <Arduino.h>

#include "config.hpp"
#include "motor_driver.hpp"
#include "homing.hpp"
#include "encoder.hpp"
namespace
{
    bool homed = false;
}
// Be warned this code is long and complex.
// It is a state machine that implements a homing sequence for a motorized mechanism with an encoder. The homing sequence consists of three phases: seeking the upper mechanical stop, backing off from the stop, and establishing a software reference position. 
// The code includes error handling for various failure modes, such as no movement, unexpected reverse movement, and timeouts. 
// It also includes debug output to the serial console to help diagnose issues during the homing process.
bool homing()
{
    /*
     * HOMING SEQUENCE
     *
     * 1. Mark the axis as unhomed.
     * 2. Drive upward towards the mechanical stop.
     * 3. Only accept encoder movement in the expected upward direction.
     * 4. Detect the stop when valid upward progress ceases for the
     *    configured stall interval.
     * 5. Stop and allow the mechanism to settle.
     * 6. Move downward by a fixed encoder distance.
     * 7. Stop, define that unloaded position as zero, and mark the
     *    homing operation successful.
     *
     * IMPORTANT LIMITATION:
     * Encoder-based stall detection cannot distinguish the intended
     * upper stop from an obstruction elsewhere in the mechanism.
     * A limit switch or current sensor would provide stronger validation.
     */

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("HOMING STARTED"));
    Serial.println(F("================================"));

    // The position must not be trusted while homing is in progress.
    homed = false;

    // Always begin from a known motor-output state.
    stopMotor();

    // Basic configuration checks. These should never fail if the
    // constants are configured correctly, but they prevent undefined
    // behaviour if a value is accidentally set to zero or negative.
    if (Config::HOME_MIN_MOVEMENT_COUNTS <= 0)
    {
        Serial.println(F("FAULT: Invalid home movement threshold"));
        return false;
    }

    if (Config::HOME_BACKOFF_COUNTS <= 0)
    {
        Serial.println(F("FAULT: Invalid home backoff distance"));
        return false;
    }

    if (Config::HOME_STALL_TIME_MS == 0 ||
        Config::HOME_TIMEOUT_MS == 0 ||
        Config::HOME_BACKOFF_TIMEOUT_MS == 0)
    {
        Serial.println(F("FAULT: Invalid homing timeout configuration"));
        return false;
    }

    // ================================================================================================
    // PHASE 1: SEEK THE UPPER MECHANICAL STOP
    // ================================================================================================

    const unsigned long homingStartTime = millis();

    unsigned long lastProgressTime = homingStartTime;
    unsigned long lastDebugTime = homingStartTime;

    const long initialPosition = getEncoderPosition();

    /*
     * lastProgressPosition is not updated for every encoder edge.
     * It is updated only after meaningful movement in the expected
     * upward direction.
     *
     * This prevents small encoder fluctuations at the mechanical stop
     * from continually resetting the stall timer.
     */
    long lastProgressPosition = initialPosition;

    bool upwardMovementObserved = false;

    Serial.print(F("Initial encoder position: "));
    Serial.println(initialPosition);

    Serial.print(F("Seeking upper stop at PWM: "));
    Serial.println(Config::MOTOR_PWM_SLOW);

    // Based on the current mechanism:
    // CCW moves upward and upward movement decreases encoder counts.
    driveMotorCCW(Config::MOTOR_PWM_SLOW);

    while (true)
    {
        const unsigned long currentTime = millis();
        const long currentPosition = getEncoderPosition();

        /*
         * EXPECTED UPWARD PROGRESS
         *
         * Upward motion decreases the encoder count.
         *
         * Example:
         *     last position    = -100
         *     current position = -107
         *
         * The mechanism has moved upward by seven counts.
         */
        if (currentPosition <=
            lastProgressPosition - Config::HOME_MIN_MOVEMENT_COUNTS)
        {
            lastProgressPosition = currentPosition;
            lastProgressTime = currentTime;
            upwardMovementObserved = true;
        }

        /*
         * UNEXPECTED REVERSE MOVEMENT
         *
         * During upward seeking, the count should not increase
         * substantially. A large increase suggests:
         *
         * - the direction convention is wrong;
         * - encoder phases have been swapped;
         * - the motor is physically moving in the wrong direction;
         * - severe encoder corruption is occurring.
         */
        // if (currentPosition >=
        //     lastProgressPosition + Config::HOME_REVERSE_MOVEMENT_LIMIT)
        // {
        //     stopMotor();

        //     Serial.println(F("FAULT: Encoder moved in unexpected direction"));
        //     Serial.print(F("Expected upward decrease from: "));
        //     Serial.println(lastProgressPosition);
        //     Serial.print(F("Observed position: "));
        //     Serial.println(currentPosition);

        //     return false;
        // }

        /*
         * RATE-LIMITED DEBUG OUTPUT
         *
         * Serial printing is deliberately limited. Printing continuously
         * can slow the loop and affect timing behaviour.
         */
        if (currentTime - lastDebugTime >=
            Config::HOME_DEBUG_INTERVAL_MS)
        {
            lastDebugTime = currentTime;

            Serial.print(F("Seeking position: "));
            Serial.print(currentPosition);

            Serial.print(F("\tLast progress: "));
            Serial.print(lastProgressPosition);

            Serial.print(F("\tNo-progress time: "));
            Serial.println(currentTime - lastProgressTime);
        }

        /*
         * STOP DETECTION
         *
         * If meaningful upward progress has ceased for the configured
         * interval, assume the upper stop has been reached.
         *
         * Requiring upwardMovementObserved reduces the risk of declaring
         * a successful home when:
         *
         * - the encoder is disconnected;
         * - the motor is disconnected;
         * - the motor never started;
         * - the mechanism was already jammed somewhere else.
         */
        if (upwardMovementObserved &&
            currentTime - lastProgressTime >=
                Config::HOME_STALL_TIME_MS)
        {
            stopMotor();

            Serial.println(F("Upper stop detected"));
            Serial.print(F("Stop encoder position: "));
            Serial.println(currentPosition);

            break;
        }

        /*
         * NO MOVEMENT FROM THE START
         *
         * If no encoder movement has ever been observed and the full
         * homing timeout expires, the firmware cannot safely determine
         * whether:
         *
         * - the lifter started against the upper stop;
         * - the motor failed to move;
         * - the encoder failed;
         * - the mechanism is jammed.
         *
         * Without a limit switch or current sensor, these cases cannot
         * be distinguished reliably. Therefore the safe response is to
         * fail rather than incorrectly establish zero.
         */
        if (!upwardMovementObserved &&
            currentTime - homingStartTime >=
                Config::HOME_TIMEOUT_MS)
        {
            stopMotor();

            Serial.println(F("FAULT: No encoder movement during homing"));
            Serial.println(F("Possible causes:"));
            Serial.println(F("- Axis already against stop"));
            Serial.println(F("- Motor not moving"));
            Serial.println(F("- Encoder disconnected"));
            Serial.println(F("- Mechanism obstructed"));

            return false;
        }

        /*
         * OVERALL SEEK TIMEOUT
         *
         * This handles a mechanism that continues moving or generating
         * encoder activity but never reaches a detectable upper stop.
         */
        if (currentTime - homingStartTime >=
            Config::HOME_TIMEOUT_MS)
        {
            stopMotor();

            Serial.println(F("FAULT: Upper-stop seek timeout"));
            Serial.print(F("Final seek position: "));
            Serial.println(currentPosition);

            return false;
        }
    }

    // Allow motor current, gearbox loading and encoder signals to settle.
    delay(Config::HOME_SETTLE_TIME_MS);

    // ================================================================================================
    // PHASE 2: BACK AWAY FROM THE MECHANICAL STOP
    // ================================================================================================

    const long backoffStartPosition = getEncoderPosition();

    /*
     * Downward motion increases encoder counts with the current wiring.
     *
     * Example:
     *     backoff start  = -948
     *     backoff target = -916
     */
    const long backoffTargetPosition =
        backoffStartPosition + Config::HOME_BACKOFF_COUNTS;

    const unsigned long backoffStartTime = millis();

    unsigned long lastBackoffProgressTime = backoffStartTime;
    unsigned long lastBackoffDebugTime = backoffStartTime;

    long lastBackoffProgressPosition = backoffStartPosition;

    bool backoffMovementObserved = false;

    Serial.println();
    Serial.println(F("Beginning home backoff"));

    Serial.print(F("Backoff start: "));
    Serial.println(backoffStartPosition);

    Serial.print(F("Backoff target: "));
    Serial.println(backoffTargetPosition);

    // CW moves the mechanism downward, away from the upper stop.
    driveMotorCW(Config::MOTOR_PWM_SLOW);

    while (true)
    {
        const unsigned long currentTime = millis();
        const long currentPosition = getEncoderPosition();

        /*
         * EXPECTED BACKOFF PROGRESS
         *
         * Downward movement increases encoder counts.
         */
        if (currentPosition >=
            lastBackoffProgressPosition +
                Config::HOME_MIN_MOVEMENT_COUNTS)
        {
            lastBackoffProgressPosition = currentPosition;
            lastBackoffProgressTime = currentTime;
            backoffMovementObserved = true;
        }

        /*
         * UNEXPECTED MOVEMENT DURING BACKOFF
         *
         * A significant encoder decrease means the mechanism appears to
         * be moving toward the stop rather than away from it.
         */
        if (currentPosition <=
            lastBackoffProgressPosition -
                Config::HOME_REVERSE_MOVEMENT_LIMIT)
        {
            stopMotor();

            Serial.println(F("FAULT: Reverse movement during backoff"));
            Serial.print(F("Last valid backoff position: "));
            Serial.println(lastBackoffProgressPosition);
            Serial.print(F("Observed position: "));
            Serial.println(currentPosition);

            return false;
        }

        if (currentTime - lastBackoffDebugTime >=
            Config::HOME_DEBUG_INTERVAL_MS)
        {
            lastBackoffDebugTime = currentTime;

            Serial.print(F("Backoff position: "));
            Serial.print(currentPosition);

            Serial.print(F("\tTarget: "));
            Serial.println(backoffTargetPosition);
        }

        /*
         * BACKOFF COMPLETE
         *
         * Use >= rather than == because the motor can move several
         * encoder counts between successive loop iterations.
         */
        if (currentPosition >= backoffTargetPosition)
        {
            stopMotor();

            Serial.print(F("Backoff final position: "));
            Serial.println(currentPosition);

            break;
        }

        /*
         * BACKOFF STALL
         *
         * The backoff has begun moving but then stopped before reaching
         * the requested target. Possible causes include another
         * obstruction or insufficient PWM.
         */
        if (backoffMovementObserved &&
            currentTime - lastBackoffProgressTime >=
                Config::HOME_STALL_TIME_MS)
        {
            stopMotor();

            Serial.println(F("FAULT: Backoff stalled before target"));
            Serial.print(F("Backoff position: "));
            Serial.println(currentPosition);
            Serial.print(F("Required target: "));
            Serial.println(backoffTargetPosition);

            return false;
        }

        /*
         * BACKOFF TIMEOUT
         *
         * This also catches the case where the motor never begins moving
         * after the direction command is issued.
         */
        if (currentTime - backoffStartTime >=
            Config::HOME_BACKOFF_TIMEOUT_MS)
        {
            stopMotor();

            if (!backoffMovementObserved)
            {
                Serial.println(F("FAULT: No encoder movement during backoff"));
            }
            else
            {
                Serial.println(F("FAULT: Homing backoff timeout"));
            }

            Serial.print(F("Backoff position: "));
            Serial.println(currentPosition);
            Serial.print(F("Backoff target: "));
            Serial.println(backoffTargetPosition);

            return false;
        }
    }

    // Allow the mechanism to unload and settle before establishing zero.
    delay(Config::HOME_SETTLE_TIME_MS);

    // ================================================================================================
    // PHASE 3: ESTABLISH THE SOFTWARE REFERENCE
    // ================================================================================================

    /*
     * Zero is defined at the backed-off, unloaded position rather than
     * directly against the mechanical stop.
     *
     * This means:
     *
     *     encoder position 0 = safe raised reference position
     *
     * rather than:
     *
     *     encoder position 0 = motor stalled against hard stop
     */
    setEncoderPosition(0);

    // Read the position back to verify that the setter took effect.
    const long zeroCheck = getEncoderPosition();

    if (zeroCheck != 0)
    {
        stopMotor();
        homed = false;

        Serial.println(F("FAULT: Encoder zero verification failed"));
        Serial.print(F("Encoder readback: "));
        Serial.println(zeroCheck);

        return false;
    }

    // The full seek, stop detection, backoff and zeroing sequence has
    // completed successfully.
    homed = true;

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("HOMING COMPLETE"));
    Serial.println(F("Encoder position set to 0"));
    Serial.println(F("================================"));

    return true;
}
