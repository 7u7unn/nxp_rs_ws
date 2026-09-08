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

TEST(ProtocolTest, UsesPublishedModelRanges)
{
  const MotorLimits & rs00 = limits_for_model(MotorModel::RS00);
  const MotorLimits & rs05 = limits_for_model(MotorModel::RS05);
  EXPECT_DOUBLE_EQ(rs00.position_rad, 4.0 * M_PI);
  EXPECT_DOUBLE_EQ(rs05.position_rad, 4.0 * M_PI);
  EXPECT_DOUBLE_EQ(rs00.velocity_rad_s, 50.0);
  EXPECT_DOUBLE_EQ(rs05.velocity_rad_s, 33.0);
}

}  // namespace rs_control
