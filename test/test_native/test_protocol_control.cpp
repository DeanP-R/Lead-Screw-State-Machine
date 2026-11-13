#include <cstdint>
#include <unity.h>

#include "control_logic.hpp"
#include "config.hpp"
#include "protocol.hpp"
#include "state_machine.hpp"
#include "mock_hardware.hpp"
#include "encoder.hpp"

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

    ControlLogic::TravelSurveyPrerequisites validSurveyPrerequisites()
    {
        ControlLogic::TravelSurveyPrerequisites p = {};
        p.enabled = true;
        p.motorDirectionVerified = true;
        p.homed = true;
        p.homingVerified = true;
        return p;
    }

    void testTravelSurveyBoundaries()
    {
        ControlLogic::TravelSurveyPrerequisites p = validSurveyPrerequisites();
        ControlLogic::TravelSurveyDecision decision =
            ControlLogic::travelSurveyMove(p, 0, -100, 100, -500, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::Accepted),
                              static_cast<int>(decision.status));
        TEST_ASSERT_EQUAL_INT32(-100, decision.targetPosition);

        decision = ControlLogic::travelSurveyMove(p, 0, 1, 100, -500, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::TowardUpperStop),
                              static_cast<int>(decision.status));
        decision = ControlLogic::travelSurveyMove(p, 0, -101, 100, -500, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::StepTooLarge),
                              static_cast<int>(decision.status));
        decision = ControlLogic::travelSurveyMove(p, -450, -51, 100, -500, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::OutsideEnvelope),
                              static_cast<int>(decision.status));
        decision = ControlLogic::travelSurveyMove(p, -500, 100, 100, -500, 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::TowardUpperStop),
                              static_cast<int>(decision.status));
    }

    void testTravelSurveyIsolation()
    {
        TEST_ASSERT_FALSE(Config::TRAVEL_SURVEY_ENABLED);
        ControlLogic::TravelSurveyPrerequisites p = validSurveyPrerequisites();
        p.enabled = false;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::Disabled),
            static_cast<int>(ControlLogic::travelSurveyMove(
                p, 0, -10, 100, -500, 0).status));

        p = validSurveyPrerequisites();
        p.travelLimitsVerified = true;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::NotReady),
            static_cast<int>(ControlLogic::travelSurveyMove(
                p, 0, -10, 100, -500, 0).status));
        p = validSurveyPrerequisites();
        p.commissioningComplete = true;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::NotReady),
            static_cast<int>(ControlLogic::travelSurveyMove(
                p, 0, -10, 100, -500, 0).status));
        p = validSurveyPrerequisites();
        p.automaticTracking = true;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::NotReady),
            static_cast<int>(ControlLogic::travelSurveyMove(
                p, 0, -10, 100, -500, 0).status));
        p = validSurveyPrerequisites();
        p.automaticHoming = true;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::NotReady),
            static_cast<int>(ControlLogic::travelSurveyMove(
                p, 0, -10, 100, -500, 0).status));
        p = validSurveyPrerequisites();
        p.homed = false;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlLogic::TravelSurveyStatus::NotReady),
            static_cast<int>(ControlLogic::travelSurveyMove(
                p, 0, -10, 100, -500, 0).status));
    }

    void testStopFromEveryMotionState()
    {
        mockResetHardware();
        const LifterState motionStates[] = {
            LifterState::Homing, LifterState::Moving, LifterState::Tracking
        };
        for (uint8_t index = 0; index < 3; ++index)
        {
            mockMarkMotorRunning();
            testSetLifterState(motionStates[index]);
            requestStop();
            TEST_ASSERT_TRUE(mockMotorStopped());
            TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Idle),
                                  static_cast<int>(getLifterState()));
        }
    }

    void testMoveStallLatchesFault()
    {
        mockResetHardware();
        initialiseStateMachine(true, 0);
        testBeginMove(-100, 0, true);
        TEST_ASSERT_FALSE(mockMotorStopped());
        updateStateMachine(500);
        TEST_ASSERT_TRUE(mockMotorStopped());
        TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Fault),
                              static_cast<int>(getLifterState()));
        TEST_ASSERT_EQUAL_INT(static_cast<int>(FaultCode::MoveStall),
                              static_cast<int>(getFaultCode()));
    }

    void testTravelSurveyTimeoutLatchesFault()
    {
        mockResetHardware();
        initialiseStateMachine(true, 0);
        testBeginMove(-500, 0, true);
        for (unsigned long now = 400; now < 3000; now += 400)
        {
            setEncoderPosition(-static_cast<long>(now / 80));
            updateStateMachine(now);
            TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Moving),
                                  static_cast<int>(getLifterState()));
        }
        setEncoderPosition(-40);
        updateStateMachine(3000);
        TEST_ASSERT_TRUE(mockMotorStopped());
        TEST_ASSERT_EQUAL_INT(static_cast<int>(LifterState::Fault),
                              static_cast<int>(getLifterState()));
        TEST_ASSERT_EQUAL_INT(static_cast<int>(FaultCode::MoveTimeout),
                              static_cast<int>(getFaultCode()));
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
    RUN_TEST(testTravelSurveyBoundaries);
    RUN_TEST(testTravelSurveyIsolation);
    RUN_TEST(testStopFromEveryMotionState);
    RUN_TEST(testMoveStallLatchesFault);
    RUN_TEST(testTravelSurveyTimeoutLatchesFault);
    return UNITY_END();
}
