// #include <config.hpp>

void readEncoder();
void driveMotorCW(uint8_t pwm);
void driveMotorCCW(uint8_t pwm);
void stopMotor();
void pulseMotorCW();
void pulseMotorCCW();
void setEncoderPosition(long position);
long getEncoderPosition();
bool readDistance(uint8_t &distanceMm);
void initialiseMotor();
