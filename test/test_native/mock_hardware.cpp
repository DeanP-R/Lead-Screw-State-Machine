#include "mock_hardware.hpp"

#include "comms.hpp"
#include "distance_sensor.hpp"
#include "encoder.hpp"
#include "homing.hpp"
#include "motor_driver.hpp"
#include "tracking.hpp"

namespace
{
    bool motorStopped = true;
    bool sensorInitialisation = true;
    long encoder = 0;
    SensorSample sample;
}

void mockResetHardware()
{
    motorStopped = true;
    sensorInitialisation = true;
    encoder = 0;
    sample = SensorSample();
}
bool mockMotorStopped() { return motorStopped; }
void mockMarkMotorRunning() { motorStopped = false; }
void mockSetSensorInitialisation(bool succeeds) { sensorInitialisation = succeeds; }

void stopMotor() { motorStopped = true; }
bool isMotorRunning() { return !motorStopped; }
MotorDirection getMotorDirection() { return MotorDirection::Stopped; }
void updateMotor(unsigned long) {}
void finishMotorStartBoost() {}
void commandMotor(MotorDirection, uint8_t, unsigned long) { motorStopped = false; }
uint8_t getAppliedPwm() { return 0; }
unsigned long getMotorCommandStartTime() { return 0; }
void initialiseMotor() {}

long getEncoderPosition() { return encoder; }
void setEncoderPosition(long value) { encoder = value; }
void readEncoder() {}
void initialiseEncoder() {}

bool initialiseDistanceSensor() { return sensorInitialisation; }
void updateDistanceSensor(unsigned long) {}
const SensorSample& getLatestSensorSample() { return sample; }
bool hasFreshValidDistance(unsigned long) { return false; }
uint16_t getFilteredDistance() { return 0; }
uint8_t getConsecutiveValidSamples() { return 0; }
unsigned long getLastValidDistanceTime() { return 0; }
int32_t getSensorDiagnosticRaw() { return 0; }
uint8_t getSensorDiagnosticFlags() { return 3; }

bool startHoming(unsigned long) { return false; }
HomingResult updateHoming(unsigned long) { return HomingResult::Cancelled; }
void cancelHoming() {}
bool isHomingActive() { return false; }
bool isHomed() { return false; }
void invalidateHome() {}

void resetTrackingMonitor() {}
bool setTrackingStandoff(uint16_t) { return true; }
uint16_t getTrackingStandoff() { return 10; }
bool recordTrackingError(uint16_t) { return true; }

bool sendPacket(Response, uint8_t, int32_t, uint8_t) { return true; }
void sendAck(uint8_t) {}
void sendNack(uint8_t, NackReason) {}
void initialiseComms() {}
PacketStatus receivePacket(Packet&, uint8_t&) { return PacketStatus::NoPacket; }
