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

TEST(MotorConfigTest, MapsAndClampsFingerMotorLimits)
{
  MotorConfig motor;
  motor.direction = 1;
  motor.position_offset = 2.0328;
  motor.position_scale = 0.4745634;
  motor.has_position_limits = true;
  motor.motor_position_min = 2.0328;
  motor.motor_position_max = 4.14;

  EXPECT_NEAR(to_joint_position(motor, 2.0328), 0.0, 1e-6);
  EXPECT_NEAR(to_joint_position(motor, 4.14), 1.0, 1e-6);
  EXPECT_NEAR(to_motor_position(motor, 0.0), 2.0328, 1e-5);
  EXPECT_NEAR(to_motor_position(motor, 1.0), 4.14, 1e-5);
  EXPECT_DOUBLE_EQ(to_motor_position(motor, -0.5), 2.0328);
  EXPECT_DOUBLE_EQ(to_motor_position(motor, 1.5), 4.14);
}

TEST(MotorConfigTest, RejectsMotorIdEqualToHostId)
{
  RobotConfig config;
  config.bus.host_id = 7;
  MotorConfig motor;
  motor.joint_name = "joint-7";
  motor.id = 7;
  config.motors.push_back(motor);

  std::string error;
  EXPECT_FALSE(validate_robot_config(config, error));
  EXPECT_NE(error.find("must differ from host_id"), std::string::npos);
}

TEST(MotorConfigTest, RejectsInvalidMotorPositionLimits)
{
  RobotConfig config;
  MotorConfig motor;
  motor.joint_name = "joint-1";
  motor.id = 1;
  motor.has_position_limits = true;
  motor.motor_position_min = 4.14;
  motor.motor_position_max = 2.0328;
  config.motors.push_back(motor);

  std::string error;
  EXPECT_FALSE(validate_robot_config(config, error));
  EXPECT_NE(error.find("strictly increasing"), std::string::npos);
}

}  // namespace rs_control
