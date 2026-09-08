#ifndef RS_CONTROL__MOTOR_CONFIG_HPP_
#define RS_CONTROL__MOTOR_CONFIG_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rs_control/robstride_bus.hpp"

namespace rs_control
{

struct RobotConfig
{
  BusConfig bus;
  bool read_only{true};
  double poll_rate_hz{20.0};
  std::vector<MotorConfig> motors;
};

bool load_robot_config(const std::string & path, RobotConfig & config, std::string & error);

bool validate_robot_config(const RobotConfig & config, std::string & error);

double to_joint_position(const MotorConfig & motor, double motor_position);

double to_joint_velocity(const MotorConfig & motor, double motor_velocity);

double to_joint_effort(const MotorConfig & motor, double motor_torque);

double to_motor_position(const MotorConfig & motor, double joint_position);

double to_motor_velocity(const MotorConfig & motor, double joint_velocity);

}  // namespace rs_control

#endif  // RS_CONTROL__MOTOR_CONFIG_HPP_
