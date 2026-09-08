#include <string>

#include <gtest/gtest.h>

#include "rs_control/motor_config.hpp"

namespace rs_control
{

TEST(MotorConfigTest, ConvertsPositionVelocityAndEffort)
{
  MotorConfig motor;
  motor.direction = -1;
  motor.position_offset = 1.5;
  motor.position_scale = 0.25;

  EXPECT_DOUBLE_EQ(to_joint_position(motor, 2.5), -0.25);
  EXPECT_DOUBLE_EQ(to_joint_velocity(motor, 4.0), -1.0);
  EXPECT_DOUBLE_EQ(to_joint_effort(motor, 3.0), -3.0);
  EXPECT_DOUBLE_EQ(to_motor_position(motor, -0.25), 2.5);
  EXPECT_DOUBLE_EQ(to_motor_velocity(motor, -1.0), 4.0);
}

TEST(MotorConfigTest, RejectsUnassignedCanId)
{
  RobotConfig config;
  MotorConfig motor;
  motor.joint_name = "joint-1";
  motor.position_scale = 1.0;
  config.motors.push_back(motor);

  std::string error;
  EXPECT_FALSE(validate_robot_config(config, error));
  EXPECT_NE(error.find("CAN id 0"), std::string::npos);
}

TEST(MotorConfigTest, RejectsDuplicateCanIds)
{
  RobotConfig config;
  MotorConfig first;
  first.joint_name = "joint-1";
  first.id = 1;
  MotorConfig second;
  second.joint_name = "joint-2";
  second.id = 1;
  config.motors = {first, second};

  std::string error;
  EXPECT_FALSE(validate_robot_config(config, error));
  EXPECT_NE(error.find("duplicate CAN id"), std::string::npos);
}

TEST(MotorConfigTest, UsesModelSpecificLimits)
{
  EXPECT_EQ(parse_motor_model("rs-00"), MotorModel::RS00);
  EXPECT_EQ(parse_motor_model("RS05"), MotorModel::RS05);
  EXPECT_FALSE(parse_motor_model("rs-99").has_value());
  EXPECT_LT(limits_for_model(MotorModel::RS05).velocity_rad_s,
    limits_for_model(MotorModel::RS00).velocity_rad_s);
}

}  // namespace rs_control
