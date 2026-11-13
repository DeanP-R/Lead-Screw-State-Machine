#include "control_logic.hpp"

namespace ControlLogic
{
    bool canTrack(const TrackingPrerequisites& p)
    {
        return p.commissioned && p.motorDirectionVerified && p.homed &&
            p.homingVerified && p.travelLimitsVerified &&
            p.feedbackDirectionVerified && p.sensorValid &&
            (p.feedbackSign == 1 || p.feedbackSign == -1);
    }

    bool withinLimits(long position, long minimum, long maximum)
    {
        return position >= minimum && position <= maximum;
    }

    long clamp(long value, long minimum, long maximum)
    {
        if (value < minimum) return minimum;
        if (value > maximum) return maximum;
        return value;
    }

    Correction trackingCorrection(
        long position, bool valid, uint16_t distanceMm, uint16_t targetMm,
        uint16_t deadbandMm, int8_t feedbackSign, long countsPerMm,
        long minimumStep, long maximumStep, long minimumPosition,
        long maximumPosition)
    {
        if (!valid) return Correction(CorrectionStatus::InvalidMeasurement, position, 0);
        if (feedbackSign != 1 && feedbackSign != -1)
            return Correction(CorrectionStatus::UnknownFeedback, position, 0);

        const int16_t error = static_cast<int16_t>(targetMm) -
            static_cast<int16_t>(distanceMm);
        if (error >= -static_cast<int16_t>(deadbandMm) &&
            error <= static_cast<int16_t>(deadbandMm))
            return Correction(CorrectionStatus::Hold, position, error);

        long magnitude = static_cast<long>(error < 0 ? -error : error) * countsPerMm;
        magnitude = clamp(magnitude, minimumStep, maximumStep);
        const int8_t distanceDirection = error > 0 ? 1 : -1;
        const long requested = position +
            static_cast<long>(distanceDirection * feedbackSign) * magnitude;
        const long bounded = clamp(requested, minimumPosition, maximumPosition);
        if (bounded == position)
            return Correction(CorrectionStatus::AtLimit, position, error);
        return Correction(CorrectionStatus::Move, bounded, error);
    }

    TravelSurveyDecision travelSurveyMove(
        const TravelSurveyPrerequisites& p, long position, long offset,
        long maximumStep, long provisionalMinimum, long provisionalMaximum)
    {
        if (!p.enabled)
            return TravelSurveyDecision(TravelSurveyStatus::Disabled, position);
        if (p.commissioningComplete || !p.motorDirectionVerified || !p.homed ||
            !p.homingVerified || p.travelLimitsVerified || p.automaticHoming ||
            p.automaticTracking)
            return TravelSurveyDecision(TravelSurveyStatus::NotReady, position);
        if (provisionalMinimum > provisionalMaximum || provisionalMaximum > 0 ||
            maximumStep <= 0)
            return TravelSurveyDecision(TravelSurveyStatus::InvalidEnvelope, position);
        // Survey motion is deliberately one-way: away from the upper stop.
        // Returning upward requires a new supervised HOME operation.
        if (offset > 0)
            return TravelSurveyDecision(TravelSurveyStatus::TowardUpperStop, position);
        if (offset == 0 || offset < -maximumStep)
            return TravelSurveyDecision(TravelSurveyStatus::StepTooLarge, position);
        if (!withinLimits(position, provisionalMinimum, provisionalMaximum))
            return TravelSurveyDecision(TravelSurveyStatus::OutsideEnvelope, position);

        const int64_t candidate = static_cast<int64_t>(position) + offset;
        if (candidate > 0)
            return TravelSurveyDecision(TravelSurveyStatus::TowardUpperStop, position);
        if (candidate < provisionalMinimum || candidate > provisionalMaximum)
            return TravelSurveyDecision(TravelSurveyStatus::OutsideEnvelope, position);
        return TravelSurveyDecision(TravelSurveyStatus::Accepted,
                                    static_cast<long>(candidate));
    }
}
