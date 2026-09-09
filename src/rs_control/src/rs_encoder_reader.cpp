#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rs_control/motor_config.hpp"

namespace rs_control
{

namespace
{

void add_diagnostic_value(
  diagnostic_msgs::msg::DiagnosticStatus & diagnostic,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  diagnostic.values.push_back(item);
}

}  // namespace

class EncoderReaderNode final : public rclcpp::Node
{
public:
  EncoderReaderNode()
  : Node("rs_encoder_reader")
  {
    const std::string config_path = declare_parameter<std::string>("config_file", "");
    const std::string joint_state_topic = declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);

    if (config_path.empty()) {
      throw std::runtime_error("parameter 'config_file' is required");
    }
    std::string error;
    if (!load_robot_config(config_path, config_, error)) {
      throw std::runtime_error(error);
    }
    bus_ = std::make_unique<RobstrideBus>(config_.bus);
    if (!bus_->connect(error)) {
      throw std::runtime_error(error);
    }
    if (publish_rate_hz <= 0.0) {
      throw std::runtime_error("parameter 'publish_rate_hz' must be positive");
    }

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(
      std::max(std::chrono::milliseconds(1), period),
      std::bind(&EncoderReaderNode::poll, this));
  }

  ~EncoderReaderNode() override
  {
    if (bus_) {
      bus_->disconnect();
    }
  }

private:
  void poll()
  {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header = joint_state.header;

    int finger_index = -1;
    bool has_valid_state = false;

    for (const auto & motor : config_.motors) {
      MotorState state;
      std::string error;
      const bool success = bus_->read_status(motor.id, motor.model, state, error);
      auto & diagnostic = diagnostics.status.emplace_back();
      diagnostic.name = "rs_control/" + motor.joint_name;
      diagnostic.hardware_id = config_.bus.interface_name + ":" + std::to_string(motor.id);
      add_diagnostic_value(diagnostic, "model", motor_model_name(motor.model));
      add_diagnostic_value(diagnostic, "can_id", std::to_string(motor.id));

      if (!success) {
        diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diagnostic.message = error;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Encoder read failed for %s: %s", motor.joint_name.c_str(), error.c_str());
        continue;
      }

      diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diagnostic.message = "operation status read OK";
      add_diagnostic_value(diagnostic, "motor_position_rad", std::to_string(state.position_rad));
      add_diagnostic_value(
        diagnostic, "motor_velocity_rad_s", std::to_string(state.velocity_rad_s));
      add_diagnostic_value(diagnostic, "motor_torque_nm", std::to_string(state.torque_nm));
      add_diagnostic_value(
        diagnostic, "motor_temperature_c", std::to_string(state.temperature_c));
      add_diagnostic_value(diagnostic, "status_flags", std::to_string(state.status_flags));
      add_diagnostic_value(diagnostic, "fault_code", std::to_string(state.fault_code));
      add_diagnostic_value(diagnostic, "warning_code", std::to_string(state.warning_code));

      joint_state.name.push_back(motor.joint_name);
      joint_state.position.push_back(to_joint_position(motor, state.position_rad));
      joint_state.velocity.push_back(to_joint_velocity(motor, state.velocity_rad_s));
      joint_state.effort.push_back(to_joint_effort(motor, state.torque_nm));
      has_valid_state = true;

      if (motor.joint_name == "joint_right-finger") {
        finger_index = static_cast<int>(joint_state.position.size() - 1);
      }
    }

    if (finger_index >= 0) {
      const auto iterator = std::find_if(
        config_.motors.begin(), config_.motors.end(),
        [](const MotorConfig & motor) { return motor.joint_name == "joint_right-finger"; });
      if (iterator != config_.motors.end()) {
        const double right_position = joint_state.position[static_cast<std::size_t>(finger_index)];
        const double right_velocity = joint_state.velocity[static_cast<std::size_t>(finger_index)];
        const double right_effort = joint_state.effort[static_cast<std::size_t>(finger_index)];
        joint_state.name.push_back("joint_left-finger");
        joint_state.position.push_back(-right_position);
        joint_state.velocity.push_back(-right_velocity);
        joint_state.effort.push_back(right_effort);
      }
    }

    if (has_valid_state) {
      joint_state_publisher_->publish(joint_state);
    }
    diagnostics_publisher_->publish(diagnostics);
  }

  RobotConfig config_;
  std::unique_ptr<RobstrideBus> bus_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rs_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<rs_control::EncoderReaderNode>());
  } catch (const std::exception & exception) {
    fprintf(stderr, "[rs_encoder_reader] %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
