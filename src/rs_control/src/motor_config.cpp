#include "rs_control/motor_config.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace rs_control
{
namespace
{

template<typename T>
bool read_optional(
  const YAML::Node & node, const char * key, T & destination, std::string & error)
{
  if (!node[key]) {
    return true;
  }

  try {
    destination = node[key].as<T>();
    return true;
  } catch (const YAML::Exception & exception) {
    error = std::string("invalid '" ) + key + "': " + exception.what();
    return false;
  }
}

}  // namespace

bool load_robot_config(const std::string & path, RobotConfig & config, std::string & error)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    error = "unable to load motor config '" + path + "': " + exception.what();
    return false;
  }

  RobotConfig loaded;
  if (!read_optional(root, "can_interface", loaded.bus.interface_name, error) ||
    !read_optional(root, "bitrate", loaded.bus.bitrate, error) ||
    !read_optional(root, "read_only", loaded.read_only, error) ||
    !read_optional(root, "poll_rate_hz", loaded.poll_rate_hz, error))
  {
    return false;
  }

  if (root["host_id"]) {
    int host_id = 0;
    try {
      host_id = root["host_id"].as<int>();
    } catch (const YAML::Exception & exception) {
      error = std::string("invalid 'host_id': ") + exception.what();
      return false;
    }
    if (host_id < 0 || host_id > 255) {
      error = "host_id must be in the range 0..255";
      return false;
    }
    loaded.bus.host_id = static_cast<uint8_t>(host_id);
  }

  if (root["response_timeout_ms"]) {
    int timeout_ms = 0;
    try {
      timeout_ms = root["response_timeout_ms"].as<int>();
    } catch (const YAML::Exception & exception) {
      error = std::string("invalid 'response_timeout_ms': ") + exception.what();
      return false;
    }
    loaded.bus.response_timeout = std::chrono::milliseconds(timeout_ms);
  }

  if (!root["motors"] || !root["motors"].IsSequence()) {
    error = "motor config must contain a sequence named 'motors'";
    return false;
  }

  for (std::size_t index = 0; index < root["motors"].size(); ++index) {
    const YAML::Node motor_node = root["motors"][index];
    MotorConfig motor;
    try {
      motor.joint_name = motor_node["joint"].as<std::string>();
      const int id = motor_node["id"].as<int>();
      if (id < 0 || id > 255) {
        error = "motor entry " + std::to_string(index) + " has an invalid CAN id";
        return false;
      }
      motor.id = static_cast<uint8_t>(id);

      const auto model = parse_motor_model(motor_node["model"].as<std::string>());
      if (!model) {
        error = "motor entry " + std::to_string(index) + " has unsupported model '" +
          motor_node["model"].as<std::string>() + "'";
        return false;
      }
      motor.model = *model;

      if (motor_node["direction"]) {
        motor.direction = motor_node["direction"].as<int>();
      }
      if (motor_node["position_offset"]) {
        motor.position_offset = motor_node["position_offset"].as<double>();
      }
      if (motor_node["position_scale"]) {
        motor.position_scale = motor_node["position_scale"].as<double>();
      }
      if (motor_node["kp"]) {
        motor.kp = motor_node["kp"].as<double>();
      }
      if (motor_node["kd"]) {
        motor.kd = motor_node["kd"].as<double>();
      }
    } catch (const YAML::Exception & exception) {
      error = "invalid motor entry " + std::to_string(index) + ": " + exception.what();
      return false;
    }
    loaded.motors.push_back(motor);
  }

  if (!validate_robot_config(loaded, error)) {
    return false;
  }

  config = std::move(loaded);
  return true;
}

bool validate_robot_config(const RobotConfig & config, std::string & error)
{
  if (config.bus.interface_name.empty()) {
    error = "CAN interface must not be empty";
    return false;
  }
  if (config.bus.bitrate == 0) {
    error = "CAN bitrate must be positive";
    return false;
  }
  if (config.bus.response_timeout.count() <= 0) {
    error = "response_timeout_ms must be positive";
    return false;
  }
  if (!std::isfinite(config.poll_rate_hz) || config.poll_rate_hz <= 0.0) {
    error = "poll_rate_hz must be positive";
    return false;
  }
  if (config.motors.empty()) {
    error = "motor config must contain at least one motor";
    return false;
  }

  std::set<std::string> joint_names;
  std::set<unsigned int> motor_ids;
  for (const auto & motor : config.motors) {
    if (motor.joint_name.empty()) {
      error = "motor joint names must not be empty";
      return false;
    }
    if (!joint_names.insert(motor.joint_name).second) {
      error = "duplicate motor joint '" + motor.joint_name + "'";
      return false;
    }
    if (motor.id == 0) {
      error = "motor '" + motor.joint_name + "' has CAN id 0; assign its physical ID";
      return false;
    }
    if (!motor_ids.insert(motor.id).second) {
      error = "duplicate CAN id " + std::to_string(motor.id);
      return false;
    }
    if (motor.direction != 1 && motor.direction != -1) {
      error = "motor '" + motor.joint_name + "' direction must be 1 or -1";
      return false;
    }
    if (!std::isfinite(motor.position_scale) || motor.position_scale <= 0.0) {
      error = "motor '" + motor.joint_name + "' position_scale must be positive";
      return false;
    }
    const MotorLimits & limits = limits_for_model(motor.model);
    if (!std::isfinite(motor.kp) || motor.kp < 0.0 || motor.kp > limits.kp) {
      error = "motor '" + motor.joint_name + "' kp is outside the model range";
      return false;
    }
    if (!std::isfinite(motor.kd) || motor.kd < 0.0 || motor.kd > limits.kd) {
      error = "motor '" + motor.joint_name + "' kd is outside the model range";
      return false;
    }
  }

  return true;
}

double to_joint_position(const MotorConfig & motor, double motor_position)
{
  return (motor_position - motor.position_offset) * static_cast<double>(motor.direction) *
    motor.position_scale;
}

double to_joint_velocity(const MotorConfig & motor, double motor_velocity)
{
  return motor_velocity * static_cast<double>(motor.direction) * motor.position_scale;
}

double to_joint_effort(const MotorConfig & motor, double motor_torque)
{
  return motor_torque * static_cast<double>(motor.direction);
}

double to_motor_position(const MotorConfig & motor, double joint_position)
{
  return joint_position * static_cast<double>(motor.direction) / motor.position_scale +
    motor.position_offset;
}

double to_motor_velocity(const MotorConfig & motor, double joint_velocity)
{
  return joint_velocity * static_cast<double>(motor.direction) / motor.position_scale;
}

}  // namespace rs_control
