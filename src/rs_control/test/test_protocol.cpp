#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "rs_control/robstride_bus.hpp"

namespace rs_control
{

TEST(ProtocolTest, ComposesRobStrideExtendedCanId)
{
  EXPECT_EQ(compose_extended_id(1, 0x1234, 5), 0x01123405U);
  EXPECT_EQ(compose_extended_id(0x21, 0xFFFF, 0xFF), 0x01FFFFFFU);
}

TEST(ProtocolTest, EncodesAndDecodesUnsignedParameters)
{
  EXPECT_EQ(encode_u16(-10.0, -10.0, 10.0), 0U);
  EXPECT_EQ(encode_u16(10.0, -10.0, 10.0), 65535U);
  EXPECT_EQ(encode_u16(100.0, -10.0, 10.0), 65535U);
  EXPECT_NEAR(decode_u16(32768U, -10.0, 10.0), 0.0, 0.001);
}

TEST(ProtocolTest, DescribesStatusAndFaultBits)
{
  const std::string status = describe_status_flags(0x2500U);
  EXPECT_NE(status.find("encoder uncalibrated"), std::string::npos);
  EXPECT_NE(status.find("overtemperature"), std::string::npos);
  EXPECT_NE(status.find("undervoltage"), std::string::npos);

  const std::string fault = describe_fault_report((1U << 7) | (1U << 16), 1U);
  EXPECT_NE(fault.find("encoder uncalibrated"), std::string::npos);
  EXPECT_NE(fault.find("A-phase overcurrent"), std::string::npos);
  EXPECT_NE(fault.find("motor overtemperature warning"), std::string::npos);
}

TEST(ProtocolTest, UsesPublishedModelRanges)
{
  const MotorLimits & rs00 = limits_for_model(MotorModel::RS00);
  const MotorLimits & rs05 = limits_for_model(MotorModel::RS05);
  EXPECT_DOUBLE_EQ(rs00.position_rad, 4.0 * M_PI);
  EXPECT_DOUBLE_EQ(rs05.position_rad, 4.0 * M_PI);
  EXPECT_DOUBLE_EQ(rs00.velocity_rad_s, 50.0);
  EXPECT_DOUBLE_EQ(rs05.velocity_rad_s, 33.0);
}

TEST(ProtocolTest, BuildsExactOperationControlFrame)
{
  const MotorLimits & limits = limits_for_model(MotorModel::RS00);
  OperationCommand command;
  command.position_rad = limits.position_rad;
  command.velocity_rad_s = -limits.velocity_rad_s;
  command.torque_nm = 0.0;
  command.kp = limits.kp;
  command.kd = 0.0;

  const ProtocolFrame frame = make_operation_frame(7, MotorModel::RS00, command);

  EXPECT_EQ(frame.id, 0x01800007U);
  EXPECT_EQ(frame.length, 8U);
  const std::array<uint8_t, 8> expected{{
    0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00}};
  EXPECT_EQ(frame.data, expected);
}

TEST(ProtocolTest, BuildsExactFeedbackAndParameterFrames)
{
  const ProtocolFrame feedback = make_feedback_request_frame(0xFF, 7);
  EXPECT_EQ(feedback.id, 0x0200FF07U);
  const std::array<uint8_t, 8> empty_data{};
  EXPECT_EQ(feedback.data, empty_data);

  const ProtocolFrame read = make_parameter_read_frame(0xFF, 7, kMechanicalPositionParameter);
  EXPECT_EQ(read.id, 0x1100FF07U);
  const std::array<uint8_t, 8> expected_read{{0x19, 0x70, 0, 0, 0, 0, 0, 0}};
  EXPECT_EQ(read.data, expected_read);

  const std::array<uint8_t, 4> mode_value{{3, 0, 0, 0}};
  const ProtocolFrame write = make_parameter_write_frame(
    0xFF, 7, kModeParameter, mode_value);
  EXPECT_EQ(write.id, 0x1200FF07U);
  const std::array<uint8_t, 8> expected_write{{0x05, 0x70, 0, 0, 3, 0, 0, 0}};
  EXPECT_EQ(write.data, expected_write);
}

TEST(ProtocolTest, BuildsExactTorqueControlFrames)
{
  const ProtocolFrame enable = make_enable_frame(0xFF, 7);
  const ProtocolFrame disable = make_disable_frame(0xFF, 7);
  EXPECT_EQ(enable.id, 0x0300FF07U);
  EXPECT_EQ(disable.id, 0x0400FF07U);
  const std::array<uint8_t, 8> empty_data{};
  EXPECT_EQ(enable.data, empty_data);
  EXPECT_EQ(disable.data, empty_data);
}

}  // namespace rs_control
