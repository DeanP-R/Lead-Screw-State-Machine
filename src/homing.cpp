#include "homing.hpp"

#include "config.hpp"
#include "encoder.hpp"
#include "motor_driver.hpp"

namespace
{
    enum class Phase : uint8_t { Idle, Seek, SeekSettle, Backoff, BackoffSettle };
    Phase phase = Phase::Idle;
    bool homed = false;
    unsigned long phaseStart = 0;
    unsigned long lastProgressTime = 0;
    long phaseStartPosition = 0;
    long lastProgressPosition = 0;

    MotorDirection directionForSign(int8_t sign)
    {
        return sign > 0 ? MotorDirection::Positive : MotorDirection::Negative;
    }

    HomingResult finish(HomingResult result)
    {
        stopMotor();
        phase = Phase::Idle;
        if (result != HomingResult::Success) homed = false;
        return result;
    }

    void beginBackoff(unsigned long now)
    {
        phase = Phase::Backoff;
        phaseStart = now;
        phaseStartPosition = getEncoderPosition();
        lastProgressPosition = phaseStartPosition;
        lastProgressTime = now;
        commandMotor(directionForSign(-Config::HOME_SEEK_ENCODER_DIRECTION),
                     Config::MOTOR_HOMING_PWM, now);
    }
}

bool startHoming(unsigned long now)
{
    if (phase != Phase::Idle ||
        (Config::HOME_SEEK_ENCODER_DIRECTION != 1 &&
         Config::HOME_SEEK_ENCODER_DIRECTION != -1)) return false;
    homed = false;
    phase = Phase::Seek;
    phaseStart = now;
    phaseStartPosition = getEncoderPosition();
    lastProgressPosition = phaseStartPosition;
    lastProgressTime = now;
    commandMotor(directionForSign(Config::HOME_SEEK_ENCODER_DIRECTION),
                 Config::MOTOR_HOMING_PWM, now);
    return true;
}

HomingResult updateHoming(unsigned long now)
{
    if (phase == Phase::Idle) return HomingResult::Cancelled;
    const long position = getEncoderPosition();

    if (phase == Phase::Seek)
    {
        const long signedTravel =
            (position - phaseStartPosition) * Config::HOME_SEEK_ENCODER_DIRECTION;
        if (now - phaseStart > Config::HOME_REVERSE_GRACE_MS &&
            signedTravel < -Config::HOME_REVERSE_LIMIT_COUNTS)
            return finish(HomingResult::SeekReverseMovement);
        if (signedTravel > Config::HOME_MAX_SEEK_COUNTS)
            return finish(HomingResult::SeekTimeout);
        if (signedTravel >= Config::HOME_INITIAL_MOVEMENT_COUNTS &&
            position != lastProgressPosition)
        {
            if ((position - lastProgressPosition) *
                Config::HOME_SEEK_ENCODER_DIRECTION >=
                Config::HOME_MIN_PROGRESS_COUNTS)
            {
                lastProgressPosition = position;
                lastProgressTime = now;
                finishMotorStartBoost();
            }
        }
        const bool moved = signedTravel >= Config::HOME_INITIAL_MOVEMENT_COUNTS;
        const unsigned long stallWindow = moved ? Config::HOME_STALL_MS
                                                 : Config::HOME_INITIAL_STALL_MS;
        if (now - lastProgressTime >= stallWindow)
        {
            stopMotor();
            phase = Phase::SeekSettle;
            phaseStart = now;
            return HomingResult::InProgress;
        }
        if (now - phaseStart >= Config::HOME_SEEK_TIMEOUT_MS)
            return finish(HomingResult::SeekTimeout);
        if (!isMotorRunning()) return finish(HomingResult::SeekTimeout);
        return HomingResult::InProgress;
    }

    if (phase == Phase::SeekSettle)
    {
        if (now - phaseStart >= Config::HOME_SETTLE_MS) beginBackoff(now);
        return HomingResult::InProgress;
    }

    if (phase == Phase::Backoff)
    {
        const long signedTravel =
            (position - phaseStartPosition) * -Config::HOME_SEEK_ENCODER_DIRECTION;
        if (now - phaseStart > Config::HOME_REVERSE_GRACE_MS &&
            signedTravel < -Config::HOME_REVERSE_LIMIT_COUNTS)
            return finish(HomingResult::BackoffReverseMovement);
        if (signedTravel >= Config::HOME_BACKOFF_COUNTS)
        {
            stopMotor();
            phase = Phase::BackoffSettle;
            phaseStart = now;
            return HomingResult::InProgress;
        }
        if ((position - lastProgressPosition) *
            -Config::HOME_SEEK_ENCODER_DIRECTION >= Config::HOME_MIN_PROGRESS_COUNTS)
        {
            lastProgressPosition = position;
            lastProgressTime = now;
            finishMotorStartBoost();
        }
        if (now - lastProgressTime >= Config::HOME_STALL_MS)
        {
            if (signedTravel < Config::HOME_INITIAL_MOVEMENT_COUNTS)
                return finish(HomingResult::BackoffNoMovement);
            return finish(HomingResult::BackoffStall);
        }
        if (now - phaseStart >= Config::HOME_BACKOFF_TIMEOUT_MS || !isMotorRunning())
            return finish(HomingResult::BackoffTimeout);
        return HomingResult::InProgress;
    }

    if (now - phaseStart < Config::HOME_SETTLE_MS)
        return HomingResult::InProgress;
    setEncoderPosition(0);
    if (getEncoderPosition() != 0) return finish(HomingResult::ZeroingFailed);
    homed = true;
    return finish(HomingResult::Success);
}

void cancelHoming() { if (phase != Phase::Idle) finish(HomingResult::Cancelled); }
bool isHomingActive() { return phase != Phase::Idle; }
bool isHomed() { return homed; }
void invalidateHome() { homed = false; }
