#include <Wire.h>
#include <VL53L0X.h>

#include "config.hpp"
#include "distance_sensor.hpp"

namespace
{
    VL53L0X sensor;
    SensorSample latest;
    bool initialised = false;
    unsigned long lastAttempt = 0;
    unsigned long lastValid = 0;
    uint16_t samples[Config::SENSOR_FILTER_SAMPLES] = {};
    uint8_t sampleCount = 0;
    uint8_t sampleIndex = 0;
    uint8_t consecutiveValid = 0;
    uint16_t filtered = 0;

    uint16_t median()
    {
        uint16_t ordered[Config::SENSOR_FILTER_SAMPLES] = {};
        for (uint8_t i = 0; i < sampleCount; ++i) ordered[i] = samples[i];
        for (uint8_t i = 1; i < sampleCount; ++i)
        {
            const uint16_t value = ordered[i];
            int8_t j = static_cast<int8_t>(i) - 1;
            while (j >= 0 && ordered[j] > value)
            {
                ordered[j + 1] = ordered[j];
                --j;
            }
            ordered[j + 1] = value;
        }
        return sampleCount == 0 ? 0 : ordered[sampleCount / 2];
    }
}

bool initialiseDistanceSensor()
{
    Wire.begin();
    initialised = sensor.init();
    if (initialised)
    {
        sensor.setTimeout(Config::SENSOR_IO_TIMEOUT_MS);
        sensor.setMeasurementTimingBudget(33000);
    }
    latest = SensorSample {};
    sampleCount = sampleIndex = consecutiveValid = 0;
    lastValid = 0;
    return initialised;
}

void updateDistanceSensor(unsigned long now)
{
    if (now - lastAttempt < Config::SENSOR_SAMPLE_INTERVAL_MS) return;
    lastAttempt = now;
    latest.timestamp = now;
    if (!initialised)
    {
        latest.valid = false;
        latest.timedOut = false;
        latest.rangeStatus = 0xFF;
        consecutiveValid = 0;
        return;
    }

    latest.distanceMm = sensor.readRangeSingleMillimeters();
    latest.timedOut = sensor.timeoutOccurred();
    latest.rangeStatus = static_cast<uint8_t>((sensor.readReg(0x14) & 0x78U) >> 3U);
    latest.valid = !latest.timedOut && latest.rangeStatus == 0 && latest.distanceMm > 0;
    if (!latest.valid)
    {
        consecutiveValid = 0;
        return;
    }

    lastValid = now;
    if (consecutiveValid < 0xFF) ++consecutiveValid;
    samples[sampleIndex] = latest.distanceMm;
    sampleIndex = (sampleIndex + 1U) % Config::SENSOR_FILTER_SAMPLES;
    if (sampleCount < Config::SENSOR_FILTER_SAMPLES) ++sampleCount;
    filtered = median();
}

const SensorSample& getLatestSensorSample() { return latest; }
bool hasFreshValidDistance(unsigned long now)
{
    return latest.valid && consecutiveValid >= Config::SENSOR_VALID_SAMPLES_REQUIRED &&
        now - lastValid <= Config::SENSOR_TIMEOUT_MS;
}
uint16_t getFilteredDistance() { return filtered; }
uint8_t getConsecutiveValidSamples() { return consecutiveValid; }
unsigned long getLastValidDistanceTime() { return lastValid; }

int32_t getSensorDiagnosticRaw()
{
    // Recovered wire contract: 0x01B40000 with range status in the low byte.
    return static_cast<int32_t>(0x01B40000UL | latest.rangeStatus);
}

uint8_t getSensorDiagnosticFlags()
{
    // Preserve observed logger values; their original bit names are unknown.
    return latest.valid ? 0x06 : 0x03;
}
