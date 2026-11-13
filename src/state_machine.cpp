#include "state_machine.hpp"

#include "comms.hpp"
#include "config.hpp"
#include "control_logic.hpp"
#include "distance_sensor.hpp"
#include "encoder.hpp"
#include "homing.hpp"
#include "motor_driver.hpp"
#include "tracking.hpp"

#include <limits.h>

namespace
{
    LifterState state = LifterState::Idle;
    FaultCode fault = FaultCode::None;
    bool telemetryEnabled = false;
    uint8_t telemetrySequence = 0;
    uint8_t operationSequence = 0;
    unsigned long lastTelemetry = 0;
    unsigned long operationStart = 0;
    unsigned long lastProgressTime = 0;
    unsigned long trackingReadyTime = 0;
    unsigned long lastTrackingSample = 0;
    long moveTarget = 0;
    long lastProgressPosition = 0;
    bool trackingMove = false;
    enum class MotionMode : uint8_t { Normal, CommissioningJog, TravelSurvey };
    MotionMode motionMode = MotionMode::Normal;
    bool startupTrackingPending = false;

    long absoluteLong(long value) { return value < 0 ? -value : value; }

    FaultCode homingFault(HomingResult result)
    {
        switch (result)
        {
            case HomingResult::InvalidConfiguration: return FaultCode::HomingInvalidConfiguration;
            case HomingResult::SeekNoMovement: return FaultCode::HomingSeekNoMovement;
            case HomingResult::SeekReverseMovement: return FaultCode::HomingSeekReverse;
            case HomingResult::SeekTimeout: return FaultCode::HomingSeekTimeout;
            case HomingResult::BackoffNoMovement: return FaultCode::HomingBackoffNoMovement;
            case HomingResult::BackoffReverseMovement: return FaultCode::HomingBackoffReverse;
            case HomingResult::BackoffStall: return FaultCode::HomingBackoffStall;
            case HomingResult::BackoffTimeout: return FaultCode::HomingBackoffTimeout;
            default: return FaultCode::HomingZeroing;
        }
    }

    void latchFault(FaultCode code)
    {
        stopMotor();
        cancelHoming();
        fault = code;
        state = LifterState::Fault;
        trackingMove = false;
        // A critical motion fault makes the reference untrustworthy.
        if (code != FaultCode::SensorTimeout &&
            code != FaultCode::TrackingDiverging) invalidateHome();
        sendPacket(Response::Fault, operationSequence, getEncoderPosition(),
                   static_cast<uint8_t>(code));
    }

    bool prerequisitesExceptSensor()
    {
        return Config::COMMISSIONING_COMPLETE && Config::MOTOR_DIRECTION_VERIFIED &&
            Config::HOMING_VERIFIED && Config::TRAVEL_LIMITS_VERIFIED &&
            Config::FEEDBACK_DIRECTION_VERIFIED &&
            (Config::DISTANCE_CHANGE_PER_POSITIVE_MOTION == 1 ||
             Config::DISTANCE_CHANGE_PER_POSITIVE_MOTION == -1);
    }

    void startMove(long target, uint8_t sequence, unsigned long now,
                   bool fromTracking, MotionMode mode = MotionMode::Normal)
    {
        moveTarget = target;
        operationSequence = sequence;
        operationStart = now;
        lastProgressTime = now;
        lastProgressPosition = getEncoderPosition();
        trackingMove = fromTracking;
        motionMode = mode;
        const MotorDirection direction = target > lastProgressPosition
            ? MotorDirection::Positive : MotorDirection::Negative;
        commandMotor(direction, Config::MOTOR_RUN_PWM, now);
    }

    bool updateMove(unsigned long now)
    {
        const long position = getEncoderPosition();
        bool positionAllowed = ControlLogic::withinLimits(
            position, Config::TRAVEL_MIN_COUNTS, Config::TRAVEL_MAX_COUNTS);
        if (motionMode == MotionMode::CommissioningJog)
            positionAllowed = absoluteLong(position) <=
                Config::COMMISSIONING_MAX_JOG_COUNTS;
        else if (motionMode == MotionMode::TravelSurvey)
            positionAllowed = ControlLogic::withinLimits(
                position, Config::TRAVEL_SURVEY_MIN_COUNTS,
                Config::TRAVEL_SURVEY_MAX_COUNTS);
        if (!positionAllowed)
        {
            latchFault(FaultCode::TravelLimit);
            return false;
        }
        const long remaining = moveTarget - position;
        const bool reached = absoluteLong(remaining) <= Config::MOVE_TOLERANCE_COUNTS ||
            (getMotorDirection() == MotorDirection::Positive && remaining < 0) ||
            (getMotorDirection() == MotorDirection::Negative && remaining > 0);
        if (reached)
        {
            stopMotor();
            if (!trackingMove)
                sendPacket(Response::MoveComplete, operationSequence, position);
            return true;
        }
        if (!isMotorRunning())
        {
            latchFault(FaultCode::MotorCommandTimeout);
            return false;
        }
        if (absoluteLong(remaining) <= Config::MOVE_DECEL_WINDOW_COUNTS)
            commandMotor(getMotorDirection(), Config::MOTOR_APPROACH_PWM, now);
        if (absoluteLong(position - lastProgressPosition) >=
            Config::MOVE_MIN_PROGRESS_COUNTS)
        {
            lastProgressPosition = position;
            lastProgressTime = now;
            finishMotorStartBoost();
        }
        const unsigned long stallTime = motionMode == MotionMode::TravelSurvey
            ? Config::TRAVEL_SURVEY_STALL_TIME_MS : Config::MOVE_STALL_TIME_MS;
        const unsigned long moveTimeout = motionMode == MotionMode::TravelSurvey
            ? Config::TRAVEL_SURVEY_TIMEOUT_MS : Config::MOVE_TIMEOUT_MS;
        if (now - lastProgressTime >= stallTime)
        {
            latchFault(FaultCode::MoveStall);
            return false;
        }
        if (now - operationStart >= moveTimeout)
        {
            latchFault(FaultCode::MoveTimeout);
            return false;
        }
        return false;
    }

    void sendTelemetry(unsigned long now)
    {
        if (!telemetryEnabled || now - lastTelemetry < Config::TELEMETRY_INTERVAL_MS)
            return;
        lastTelemetry = now;
        const SensorSample& sample = getLatestSensorSample();
        const uint8_t stateFlag = static_cast<uint8_t>(state) & 0x0F;
        const uint8_t distanceFlags = stateFlag | (sample.valid ? 0x80 : 0x00);
        sendPacket(Response::Distance, telemetrySequence,
                   sample.valid ? static_cast<int32_t>(sample.distanceMm) : -1,
                   distanceFlags);
        sendPacket(Response::Encoder, telemetrySequence, getEncoderPosition(), stateFlag);
        sendPacket(Response::SensorDiagnostic, telemetrySequence,
                   getSensorDiagnosticRaw(), getSensorDiagnosticFlags());
        ++telemetrySequence;
    }

    void updateTracking(unsigned long now)
    {
        if (!prerequisitesExceptSensor() || !isHomed())
        {
            latchFault(FaultCode::TravelLimit);
            return;
        }
        if (trackingMove)
        {
            if (updateMove(now) && state != LifterState::Fault)
            {
                trackingMove = false;
                trackingReadyTime = now + Config::TRACKING_SETTLE_MS;
            }
            return;
        }
        if (!hasFreshValidDistance(now))
        {
            stopMotor();
            const unsigned long lastValid = getLastValidDistanceTime();
            if ((lastValid == 0 && now - operationStart >= Config::SENSOR_TIMEOUT_MS) ||
                (lastValid != 0 && now - lastValid >= Config::SENSOR_TIMEOUT_MS))
                latchFault(FaultCode::SensorTimeout);
            return;
        }
        const SensorSample& sample = getLatestSensorSample();
        if (now < trackingReadyTime || sample.timestamp == lastTrackingSample) return;
        lastTrackingSample = sample.timestamp;
        const ControlLogic::Correction correction = ControlLogic::trackingCorrection(
            getEncoderPosition(), true, getFilteredDistance(), getTrackingStandoff(),
            Config::TRACKING_DEADBAND_MM,
            Config::DISTANCE_CHANGE_PER_POSITIVE_MOTION,
            Config::TRACKING_COUNTS_PER_MM,
            Config::TRACKING_MIN_CORRECTION_COUNTS,
            Config::TRACKING_MAX_CORRECTION_COUNTS,
            Config::TRAVEL_MIN_COUNTS, Config::TRAVEL_MAX_COUNTS);
        if (correction.status == ControlLogic::CorrectionStatus::Hold)
        {
            stopMotor();
            resetTrackingMonitor();
            return;
        }
        if (correction.status == ControlLogic::CorrectionStatus::AtLimit)
        {
            latchFault(FaultCode::TrackingAtLimit);
            return;
        }
        if (correction.status != ControlLogic::CorrectionStatus::Move) return;
        const uint16_t absoluteError = static_cast<uint16_t>(
            correction.errorMm < 0 ? -correction.errorMm : correction.errorMm);
        if (!recordTrackingError(absoluteError))
        {
            latchFault(FaultCode::TrackingDiverging);
            return;
        }
        startMove(correction.targetPosition, operationSequence, now, true);
    }
}

void initialiseStateMachine(bool sensorInitialised, unsigned long now)
{
    stopMotor();
    invalidateHome();
    resetTrackingMonitor();
    telemetryEnabled = false;
    fault = FaultCode::None;
    state = LifterState::Idle;
    lastTelemetry = now;
    if (!sensorInitialised)
    {
        operationSequence = 0;
        latchFault(FaultCode::SensorInitialisation);
        return;
    }
    if (Config::AUTO_HOME_ON_STARTUP && Config::COMMISSIONING_COMPLETE &&
        Config::MOTOR_DIRECTION_VERIFIED) requestHoming(0, now);
    startupTrackingPending = Config::AUTO_TRACK_ON_STARTUP &&
        prerequisitesExceptSensor();
    if (startupTrackingPending && state == LifterState::Idle && !isHomed())
        requestHoming(0, now);
}

void updateStateMachine(unsigned long now)
{
    updateMotor(now);
    updateDistanceSensor(now);
    if (state == LifterState::Homing)
    {
        const HomingResult result = updateHoming(now);
        if (result == HomingResult::Success)
        {
            state = LifterState::Idle;
            sendPacket(Response::HomeComplete, operationSequence, getEncoderPosition());
        }
        else if (result != HomingResult::InProgress)
            latchFault(homingFault(result));
    }
    else if (state == LifterState::Moving)
    {
        if (updateMove(now) && state != LifterState::Fault) state = LifterState::Idle;
    }
    else if (state == LifterState::Tracking) updateTracking(now);
    else if (state == LifterState::Idle && startupTrackingPending && isHomed() &&
             hasFreshValidDistance(now))
    {
        startupTrackingPending = false;
        requestStartTracking(0, now);
    }
    sendTelemetry(now);
}

RequestResult requestHoming(uint8_t sequence, unsigned long now)
{
    if (state == LifterState::Fault) return RequestResult::InvalidState;
    if (state != LifterState::Idle) return RequestResult::Busy;
    if (!Config::MOTOR_DIRECTION_VERIFIED) return RequestResult::NotCommissioned;
    operationSequence = sequence;
    if (!startHoming(now)) return RequestResult::InvalidState;
    state = LifterState::Homing;
    return RequestResult::Accepted;
}

RequestResult requestMoveAbsolute(uint8_t sequence, long target, unsigned long now)
{
    if (state == LifterState::Fault) return RequestResult::InvalidState;
    if (state != LifterState::Idle) return RequestResult::Busy;
    if (!isHomed()) return RequestResult::NotHomed;
    if (!Config::TRAVEL_LIMITS_VERIFIED) return RequestResult::NotCommissioned;
    if (!ControlLogic::withinLimits(target, Config::TRAVEL_MIN_COUNTS,
                                    Config::TRAVEL_MAX_COUNTS))
        return RequestResult::OutOfRange;
    if (absoluteLong(target - getEncoderPosition()) <= Config::MOVE_TOLERANCE_COUNTS)
    {
        sendPacket(Response::MoveComplete, sequence, getEncoderPosition());
        return RequestResult::Accepted;
    }
    state = LifterState::Moving;
    startMove(target, sequence, now, false);
    return RequestResult::Accepted;
}

RequestResult requestMoveRelative(uint8_t sequence, long offset, unsigned long now)
{
    if (!isHomed() && Config::COMMISSIONING_JOG_ENABLED)
    {
        if (state != LifterState::Idle) return RequestResult::Busy;
        if (offset == 0 || absoluteLong(offset) > Config::COMMISSIONING_MAX_JOG_COUNTS)
            return RequestResult::OutOfRange;
        // The temporary encoder origin bounds a supervised commissioning jog.
        setEncoderPosition(0);
        state = LifterState::Moving;
        startMove(offset, sequence, now, false, MotionMode::CommissioningJog);
        return RequestResult::Accepted;
    }
    if (!isHomed()) return RequestResult::NotHomed;

    if (Config::TRAVEL_SURVEY_ENABLED && !Config::TRAVEL_LIMITS_VERIFIED)
    {
        ControlLogic::TravelSurveyPrerequisites prerequisites = {};
        prerequisites.enabled = Config::TRAVEL_SURVEY_ENABLED;
        prerequisites.commissioningComplete = Config::COMMISSIONING_COMPLETE;
        prerequisites.motorDirectionVerified = Config::MOTOR_DIRECTION_VERIFIED;
        prerequisites.homed = isHomed();
        prerequisites.homingVerified = Config::HOMING_VERIFIED;
        prerequisites.travelLimitsVerified = Config::TRAVEL_LIMITS_VERIFIED;
        prerequisites.automaticHoming = Config::AUTO_HOME_ON_STARTUP;
        prerequisites.automaticTracking = Config::AUTO_TRACK_ON_STARTUP;
        const ControlLogic::TravelSurveyDecision decision =
            ControlLogic::travelSurveyMove(
                prerequisites, getEncoderPosition(), offset,
                Config::TRAVEL_SURVEY_MAX_STEP_COUNTS,
                Config::TRAVEL_SURVEY_MIN_COUNTS,
                Config::TRAVEL_SURVEY_MAX_COUNTS);
        if (decision.status == ControlLogic::TravelSurveyStatus::Accepted)
        {
            state = LifterState::Moving;
            startMove(decision.targetPosition, sequence, now, false,
                      MotionMode::TravelSurvey);
            return RequestResult::Accepted;
        }
        if (decision.status == ControlLogic::TravelSurveyStatus::StepTooLarge ||
            decision.status == ControlLogic::TravelSurveyStatus::OutsideEnvelope ||
            decision.status == ControlLogic::TravelSurveyStatus::TowardUpperStop ||
            decision.status == ControlLogic::TravelSurveyStatus::InvalidEnvelope)
            return RequestResult::OutOfRange;
        return RequestResult::NotCommissioned;
    }

    const long position = getEncoderPosition();
    const int64_t candidate = static_cast<int64_t>(position) + offset;
    if (candidate < LONG_MIN || candidate > LONG_MAX)
        return RequestResult::OutOfRange;
    const long target = static_cast<long>(candidate);
    return requestMoveAbsolute(sequence, target, now);
}

RequestResult requestStartTracking(uint8_t sequence, unsigned long now)
{
    if (state == LifterState::Fault) return RequestResult::InvalidState;
    if (state != LifterState::Idle) return RequestResult::Busy;
    if (!prerequisitesExceptSensor()) return RequestResult::NotCommissioned;
    if (!isHomed()) return RequestResult::NotHomed;
    if (!hasFreshValidDistance(now)) return RequestResult::SensorInvalid;
    operationSequence = sequence;
    operationStart = now;
    trackingReadyTime = now;
    lastTrackingSample = 0;
    trackingMove = false;
    resetTrackingMonitor();
    state = LifterState::Tracking;
    return RequestResult::Accepted;
}

RequestResult requestStopTracking()
{
    if (state != LifterState::Tracking) return RequestResult::InvalidState;
    stopMotor();
    trackingMove = false;
    state = LifterState::Idle;
    return RequestResult::Accepted;
}

void requestStop()
{
    stopMotor();
    cancelHoming();
    trackingMove = false;
    startupTrackingPending = false;
    if (state != LifterState::Fault) state = LifterState::Idle;
}

bool clearFault()
{
    if (state != LifterState::Fault) return false;
    stopMotor();
    if (fault == FaultCode::SensorInitialisation && !initialiseDistanceSensor())
        return false;
    fault = FaultCode::None;
    state = LifterState::Idle;
    invalidateHome();
    resetTrackingMonitor();
    return true;
}

void setTelemetryEnabled(bool enabled) { telemetryEnabled = enabled; }
bool isTelemetryEnabled() { return telemetryEnabled; }
LifterState getLifterState() { return state; }
FaultCode getFaultCode() { return fault; }

#ifdef UNIT_TEST
void testBeginMove(long target, unsigned long now, bool travelSurvey)
{
    state = LifterState::Moving;
    startMove(target, 0, now, false,
              travelSurvey ? MotionMode::TravelSurvey : MotionMode::Normal);
}

void testSetLifterState(LifterState newState) { state = newState; }
#endif
