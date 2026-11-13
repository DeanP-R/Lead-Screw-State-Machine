#include <cstdint>
#include <unity.h>

#include "control_logic.hpp"
#include "protocol.hpp"
#include "state_machine.hpp"
#include "mock_hardware.hpp"

namespace
{
    void testKnownLoggerFrame()
    {
        const Protocol::Packet packet(0x03, 0x2A, -1234, 0x5A);
        uint8_t frame[Protocol::PacketLength] = {};
        Protocol::encode(packet, frame);
        TEST_ASSERT_EQUAL_HEX8(0xAA, frame[0]);
        TEST_ASSERT_EQUAL_HEX8(0x55, frame[9]);
        TEST_ASSERT_EQUAL_HEX8(0x03, frame[1]);
        TEST_ASSERT_EQUAL_UINT8(0x2A, frame[2]);
        TEST_ASSERT_EQUAL_HEX8(0x2E, frame[3]);
        TEST_ASSERT_EQUAL_HEX8(0xFB, frame[4]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, frame[5]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, frame[6]);
        TEST_ASSERT_EQUAL_HEX8(0x5A, frame[7]);
        TEST_ASSERT_EQUAL_HEX8(Protocol::crc8(&frame[1], 7), frame[8]);

        Protocol::Packet decoded;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Protocol::DecodeStatus::Valid),
                              static_cast<int>(Protocol::decode(frame, decoded)));
        TEST_ASSERT_EQUAL_UINT8(packet.command, decoded.command);
        TEST_ASSERT_EQUAL_UINT8(packet.sequence, decoded.sequence);
        TEST_ASSERT_EQUAL_INT32(packet.value, decoded.value);
        TEST_ASSERT_EQUAL_UINT8(packet.flags, decoded.flags);
        frame[8] ^= 1;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Protocol::DecodeStatus::InvalidCrc),
                              static_cast<int>(Protocol::decode(frame, decoded)));
    }

    ControlLogic::TrackingPrerequisites validPrerequisites()
    {
        ControlLogic::TrackingPrerequisites p = {};
        p.commissioned = p.motorDirectionVerified = p.homed = true;
        p.homingVerified = p.travelLimitsVerified = true;
        p.feedbackDirectionVerified = p.sensorValid = true;
        p.feedbackSign = 1;
        return p;
    }

    void testTrackingSafetyGates()
    {
        ControlLogic::TrackingPrerequisites p = validPrerequisites();
        TEST_ASSERT_TRUE(ControlLogic::canTrack(p));
        p.sensorValid = false;
        TEST_ASSERT_FALSE(ControlLogic::canTrack(p));
        p.sensorValid = true;
        p.feedbackSign = 0;
        TEST_ASSERT_FALSE(ControlLogic::canTrack(p));
    }

    void testFeedbackDirectionAndBounds()
    {
        ControlLogic::Correction c = ControlLogic::trackingCorrection(
            -500, true, 20, 10, 2, 1, 10, 5, 40, -1000, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::CorrectionStatus::Move),
                              static_cast<int>(c.status));
        TEST_ASSERT_EQUAL_INT32(-540, c.targetPosition);

        c = ControlLogic::trackingCorrection(
            -500, true, 20, 10, 2, -1, 10, 5, 40, -1000, 0);
        TEST_ASSERT_EQUAL_INT32(-460, c.targetPosition);

        c = ControlLogic::trackingCorrection(
            -500, false, 0, 10, 2, 1, 10, 5, 40, -1000, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::CorrectionStatus::InvalidMeasurement),
                              static_cast<int>(c.status));

        c = ControlLogic::trackingCorrection(
            -1000, true, 20, 10, 2, 1, 10, 5, 40, -1000, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::CorrectionStatus::AtLimit),
                              static_cast<int>(c.status));

        c = ControlLogic::trackingCorrection(
            -500, true, 11, 10, 2, 1, 10, 5, 40, -1000, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::CorrectionStatus::Hold),
                              static_cast<int>(c.status));
    }

    void testSafeStartupStopAndRecovery()
    {
        mockResetHardware();
        initialiseStateMachine(true, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Idle),
                              static_cast<int>(getLifterState()));
        TEST_ASSERT_TRUE(mockMotorStopped());
        TEST_ASSERT_EQUAL_INT(static_cast<int>(RequestResult::NotCommissioned),
                              static_cast<int>(requestHoming(1, 0)));
        TEST_ASSERT_EQUAL_INT(static_cast<int>(RequestResult::NotCommissioned),
                              static_cast<int>(requestStartTracking(2, 1000)));
        requestStop();
        TEST_ASSERT_TRUE(mockMotorStopped());
        TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Idle),
                              static_cast<int>(getLifterState()));

        initialiseStateMachine(false, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Fault),
                              static_cast<int>(getLifterState()));
        requestStop();
        TEST_ASSERT_TRUE(mockMotorStopped());
        TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Fault),
                              static_cast<int>(getLifterState()));
        mockSetSensorInitialisation(true);
        TEST_ASSERT_TRUE(clearFault());
        TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Idle),
                              static_cast<int>(getLifterState()));
    }
}

void setUp() {}
void tearDown() {}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(testKnownLoggerFrame);
    RUN_TEST(testTrackingSafetyGates);
    RUN_TEST(testFeedbackDirectionAndBounds);
    RUN_TEST(testSafeStartupStopAndRecovery);
    return UNITY_END();
}
