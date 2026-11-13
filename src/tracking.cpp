#include "tracking.hpp"

#include "config.hpp"

namespace
{
    uint16_t target = Config::TARGET_STANDOFF_MM;
    uint16_t previousError = 0xFFFF;
    uint8_t nonImproving = 0;
}

void resetTrackingMonitor()
{
    previousError = 0xFFFF;
    nonImproving = 0;
}

bool setTrackingStandoff(uint16_t millimetres)
{
    if (millimetres == 0 || millimetres > 2000) return false;
    target = millimetres;
    resetTrackingMonitor();
    return true;
}

uint16_t getTrackingStandoff() { return target; }

bool recordTrackingError(uint16_t error)
{
    if (previousError != 0xFFFF &&
        error + Config::TRACKING_MIN_IMPROVEMENT_MM >= previousError)
    {
        if (nonImproving < 0xFF) ++nonImproving;
    }
    else nonImproving = 0;
    previousError = error;
    return nonImproving < Config::TRACKING_MAX_NON_IMPROVING_CORRECTIONS;
}
