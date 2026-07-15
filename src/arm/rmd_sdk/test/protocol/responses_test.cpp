/**
 * \file responses.cpp
 * \mainpage
 *    Tests for parsing of different response messages
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#include <gtest/gtest.h>

#include "rmd_sdk/actuator_state/control_mode.hpp"
#include "rmd_sdk/actuator_state/feedback.hpp"
#include "rmd_sdk/actuator_state/gains.hpp"
#include "rmd_sdk/actuator_state/motor_status_1.hpp"
#include "rmd_sdk/actuator_state/motor_status_2.hpp"
#include "rmd_sdk/actuator_state/motor_status_3.hpp"
#include "rmd_sdk/protocol/responses.hpp"


namespace rmd_sdk {
  namespace test {

    TEST(GetCanIdResponseTest, parsing) {
      rmd_sdk::GetCanIdResponse const request {{0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x02}};
      std::uint16_t const can_id {request.getCanId()};
      EXPECT_EQ(can_id, 0x242);
    }

    TEST(GetAccelerationResponseTest, parsing) {
      rmd_sdk::GetAccelerationResponse const response {{0x42, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      std::int32_t const acceleration {response.getAcceleration()};
      EXPECT_EQ(acceleration, 10000);
    }

    TEST(GetControllerGainsResponseTest, parsing) {
      rmd_sdk::GetControllerGainsResponse const response {{0x30, 0x00, 0x55, 0x19, 0x55, 0x19, 0x55, 0x19}};
      rmd_sdk::Gains const gains {response.getGains()};
      EXPECT_EQ(gains.current.kp, 85);
      EXPECT_EQ(gains.current.ki, 25);
      EXPECT_EQ(gains.speed.kp, 85);
      EXPECT_EQ(gains.speed.ki, 25);
      EXPECT_EQ(gains.position.kp, 85);
      EXPECT_EQ(gains.position.ki, 25);
    }

    TEST(GetControlModeResponseTest, parsing) {
      rmd_sdk::GetControlModeResponse const response {{0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03}};
      rmd_sdk::ControlMode const control_mode {response.getMode()};
      EXPECT_EQ(control_mode, ControlMode::POSITION);
    }

    TEST(GetMotorModelResponseTest, parsing) {
      rmd_sdk::GetMotorModelResponse const response {{0xB5, 0x58, 0x38, 0x53, 0x32, 0x56, 0x31, 0x30}};
      std::string const version {response.getModel()};
      EXPECT_EQ(version, "X8S2V10");
    }

    TEST(GetMotorPowerResponseTest, parsing) {
      rmd_sdk::GetMotorPowerResponse const response {{0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x07}};
      auto const motor_power {response.getPower()};
      EXPECT_NEAR(motor_power, 200.0f, 0.1f);
    }

    TEST(GetMotorStatus1ResponseTest, parsing) {
      rmd_sdk::GetMotorStatus1Response const response {{0x9A, 0x32, 0x00, 0x01, 0xE5, 0x01, 0x04, 0x00}};
      rmd_sdk::MotorStatus1 const motor_status {response.getStatus()};
      EXPECT_EQ(motor_status.temperature, 50);
      EXPECT_EQ(motor_status.is_brake_released, true);
      EXPECT_NEAR(motor_status.voltage, 48.5, 0.1);
      EXPECT_EQ(motor_status.error_code, rmd_sdk::ErrorCode::LOW_VOLTAGE);
    }

    TEST(GetMotorStatus2ResponseTest, parsing) {
      rmd_sdk::GetMotorStatus2Response const response {{0x9C, 0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00}};
      rmd_sdk::MotorStatus2 const motor_status {response.getStatus()};
      EXPECT_EQ(motor_status.temperature, 50);
      EXPECT_NEAR(motor_status.current, 1.0f, 0.1f);
      EXPECT_NEAR(motor_status.shaft_speed, 500.0f, 0.1f);
      EXPECT_NEAR(motor_status.shaft_angle, 45.0f, 0.1f);
    }

    TEST(GetMotorStatus3ResponseTest, parsing) {
      rmd_sdk::GetMotorStatus3Response const response {{0x9D, 0x32, 0xC2, 0x0B, 0x10, 0xFA, 0xC0, 0xF9}};
      rmd_sdk::MotorStatus3 const motor_status {response.getStatus()};
      EXPECT_EQ(motor_status.temperature, 50);
      EXPECT_NEAR(motor_status.current_phase_a, 30.1f, 0.1f);
      EXPECT_NEAR(motor_status.current_phase_b, -15.2f, 0.1f);
      EXPECT_NEAR(motor_status.current_phase_c, -16.0f, 0.1f);
    }

    TEST(GetMultiTurnAngleResponseTest, parsing) {
      rmd_sdk::GetMultiTurnAngleResponse const response {{0x92, 0x00, 0x00, 0x00, 0xA0, 0x8C, 0x00, 0x00}};
      auto const angle {response.getAngle()};
      EXPECT_NEAR(angle, 360.0f, 0.1f);
    }

    TEST(GetMultiTurnEncoderPositionResponseTest, parsing) {
      rmd_sdk::GetMultiTurnEncoderPositionResponse const response {{0x60, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      auto const encoder_position {response.getPosition()};
      EXPECT_EQ(encoder_position, 10000);
    }

    TEST(GetMultiTurnEncoderOriginalPositionResponseTest, parsing) {
      rmd_sdk::GetMultiTurnEncoderOriginalPositionResponse const response {{0x61, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      auto const encoder_position {response.getPosition()};
      EXPECT_EQ(encoder_position, 10000);
    }

    TEST(GetMultiTurnEncoderZeroOffsetResponseTest, parsing) {
      rmd_sdk::GetMultiTurnEncoderZeroOffsetResponse const response {{0x62, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      auto const encoder_position {response.getPosition()};
      EXPECT_EQ(encoder_position, 10000);
    }

    TEST(GetSingleTurnAngleResponseTest, parsing) {
      rmd_sdk::GetSingleTurnAngleResponse const response {{0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27}};
      auto const angle {response.getAngle()};
      EXPECT_NEAR(angle, 100.0f, 0.1f);
    }

    TEST(GetSingleTurnEncoderPositionResponseTest, parsing) {
      rmd_sdk::GetSingleTurnEncoderPositionResponse const response {{0x90, 0x00, 0x33, 0x08, 0xBE, 0x2C, 0x8B, 0x24}};
      auto const encoder_position {response.getPosition()};
      EXPECT_EQ(encoder_position, 2099);
      auto const encoder_raw_position {response.getRawPosition()};
      EXPECT_EQ(encoder_raw_position, 11454);
      auto const encoder_offset {response.getOffset()};
      EXPECT_EQ(encoder_offset, 9355);
    }

    TEST(GetSystemRuntimeResponseTest, parsing) {
      rmd_sdk::GetSystemRuntimeResponse const response {{0xB1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10}};
      auto const runtime {response.getRuntime()};
      EXPECT_EQ(runtime.count(), 268435456);
    }

    TEST(GetVersionDateResponseTest, parsing) {
      rmd_sdk::GetVersionDateResponse const response {{0xB2, 0x00, 0x00, 0x00, 0x2E, 0x89, 0x34, 0x01}};
      auto const version {response.getVersion()};
      EXPECT_EQ(version, 20220206);
    }

    TEST(SetCurrentPositionAsEncoderZeroResponseTest, parsing) {
      rmd_sdk::SetCurrentPositionAsEncoderZeroResponse const response {{0x64, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      auto const encoder_zero {response.getEncoderZero()};
      EXPECT_EQ(encoder_zero, 10000);
    }

    TEST(SetPositionAbsoluteResponseTest, parsingPositiveValues) {
      rmd_sdk::SetPositionAbsoluteResponse const response {{0xA4, 0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00}};
      rmd_sdk::Feedback const feedback {response.getStatus()};
      EXPECT_EQ(feedback.temperature, 50);
      EXPECT_NEAR(feedback.current, 1.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_speed, 500.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_angle, 45.0f, 0.1f);
    }

    TEST(SetPositionAbsoluteResponseTest, parsingNegativeValues) {
      rmd_sdk::SetPositionAbsoluteResponse const response {{0xA4, 0x32, 0x9C, 0xFF, 0x0C, 0xFE, 0xD3, 0xFF}};
      rmd_sdk::Feedback const feedback {response.getStatus()};
      EXPECT_EQ(feedback.temperature, 50);
      EXPECT_NEAR(feedback.current, -1.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_speed, -500.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_angle, -45.0f, 0.1f);
    }

    TEST(SetTorqueResponseTest, parsingPositiveValues) {
      rmd_sdk::SetTorqueResponse const response {{0xA1, 0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00}};
      rmd_sdk::Feedback const feedback {response.getStatus()};
      EXPECT_EQ(feedback.temperature, 50);
      EXPECT_NEAR(feedback.current, 1.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_speed, 500.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_angle, 45.0f, 0.1f);
    }

    TEST(SetTorqueResponseTest, parsingNegativeValues) {
      rmd_sdk::SetTorqueResponse const response {{0xA1, 0x32, 0x9C, 0xFF, 0x0C, 0xFE, 0xD3, 0xFF}};
      rmd_sdk::Feedback const feedback {response.getStatus()};
      EXPECT_EQ(feedback.temperature, 50);
      EXPECT_NEAR(feedback.current, -1.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_speed, -500.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_angle, -45.0f, 0.1f);
    }

    TEST(SetVelocityResponseTest, parsingPositiveValues) {
      rmd_sdk::SetVelocityResponse const response {{0xA2, 0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00}};
      rmd_sdk::Feedback const feedback {response.getStatus()};
      EXPECT_EQ(feedback.temperature, 50);
      EXPECT_NEAR(feedback.current, 1.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_speed, 500.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_angle, 45.0f, 0.1f);
    }

    TEST(SetVelocityResponseTest, parsingNegativeValues) {
      rmd_sdk::SetVelocityResponse const response {{0xA2, 0x32, 0x9C, 0xFF, 0x0C, 0xFE, 0xD3, 0xFF}};
      rmd_sdk::Feedback const feedback {response.getStatus()};
      EXPECT_EQ(feedback.temperature, 50);
      EXPECT_NEAR(feedback.current, -1.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_speed, -500.0f, 0.1f);
      EXPECT_NEAR(feedback.shaft_angle, -45.0f, 0.1f);
    }

  }
}
