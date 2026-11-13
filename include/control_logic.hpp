#pragma once

#include <stdint.h>

namespace ControlLogic
{
    struct TrackingPrerequisites
    {
        bool commissioned;
        bool motorDirectionVerified;
        bool homed;
        bool homingVerified;
        bool travelLimitsVerified;
        bool feedbackDirectionVerified;
        bool sensorValid;
        int8_t feedbackSign;
    };

    enum class CorrectionStatus : uint8_t
    {
        Hold, Move, InvalidMeasurement, UnknownFeedback, AtLimit
    };

    struct TravelSurveyPrerequisites
    {
        bool enabled;
        bool commissioningComplete;
        bool motorDirectionVerified;
        bool homed;
        bool homingVerified;
        bool travelLimitsVerified;
        bool automaticHoming;
        bool automaticTracking;
    };

    enum class TravelSurveyStatus : uint8_t
    {
        Accepted, Disabled, NotReady, InvalidEnvelope, StepTooLarge,
        OutsideEnvelope, TowardUpperStop
    };

    struct TravelSurveyDecision
    {
        TravelSurveyStatus status;
        long targetPosition;

        TravelSurveyDecision(TravelSurveyStatus decisionStatus, long target)
            : status(decisionStatus), targetPosition(target) {}
    };

    struct Correction
    {
        CorrectionStatus status;
        long targetPosition;
        int16_t errorMm;

        Correction(CorrectionStatus correctionStatus, long position,
                   int16_t error)
            : status(correctionStatus), targetPosition(position),
              errorMm(error) {}
    };

    bool canTrack(const TrackingPrerequisites& prerequisites);
    bool withinLimits(long position, long minimum, long maximum);
    long clamp(long value, long minimum, long maximum);
    Correction trackingCorrection(
        long position, bool measurementValid, uint16_t distanceMm,
        uint16_t targetMm, uint16_t deadbandMm, int8_t feedbackSign,
        long countsPerMm, long minimumStep, long maximumStep,
        long minimumPosition, long maximumPosition);
    TravelSurveyDecision travelSurveyMove(
        const TravelSurveyPrerequisites& prerequisites,
        long position, long offset, long maximumStep,
        long provisionalMinimum, long provisionalMaximum);
}
