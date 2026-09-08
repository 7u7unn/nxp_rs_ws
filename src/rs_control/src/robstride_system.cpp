#include "rs_control/robstride_system.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rs_control
{
namespace
{

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & interface) {
      return interface.name == name;
    });
}

bool parse_bool(const std::string & value, bool & result)
{
  if (value == "true" || value == "1" || value == "yes") {
    result = true;
    return true;
  }
  if (value == "false" || value == "0" || value == "no") {
    result = false;
    return true;
  }
  return false;
}

template<typename T>
bool parse_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  T & destination,
  std::string & error)
{
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end()) {
    return true;
  }
  try {
    if constexpr (std::is_same_v<T, bool>) {
      if (!parse_bool(iterator->second, destination)) {
        error = "hardware parameter '" + name + "' must be boolean";
        return false;
      }
    } else if constexpr (std::is_same_v<T, std::string>) {
      destination = iterator->second;
    } else {
      if constexpr (std::is_integral_v<T>) {
        const long long parsed = std::stoll(iterator->second);
        if (parsed < 0 && std::is_unsigned_v<T>) {
          error = "hardware parameter '" + name + "' must not be negative";
          return false;
        }
        if (static_cast<unsigned long long>(parsed) >
          static_cast<unsigned long long>(std::numeric_limits<T>::max()))
        {
          error = "hardware parameter '" + name + "' is out of range";
          return false;
        }
        destination = static_cast<T>(parsed);
      } else {
        destination = static_cast<T>(std::stod(iterator->second));
      }
    }
  } catch (const std::exception & exception) {
    error = "invalid hardware parameter '" + name + "': " + exception.what();
    return false;
  }
  return true;
}

}  // namespace

RobstrideSystem::~RobstrideSystem()
{
  stop_io_thread();
  if (bus_) {
    disable_all_motors();
    bus_->disconnect();
  }
}

hardware_interface::CallbackReturn RobstrideSystem::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string error;
  if (!load_hardware_config(error)) {
    std::cerr << "[rs_control] " << error << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!validate_hardware_joints(error)) {
    std::cerr << "[rs_control] " << error << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t joint_count = config_.motors.size();
  state_positions_.assign(joint_count, 0.0);
  state_velocities_.assign(joint_count, 0.0);
  state_efforts_.assign(joint_count, 0.0);
  command_positions_.assign(joint_count, 0.0);
  pending_commands_.assign(joint_count, 0.0);
  latest_states_.assign(joint_count, MotorState{});
  bus_ = std::make_unique<RobstrideBus>(config_.bus);
  initialized_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RobstrideSystem::load_hardware_config(std::string & error)
{
  const auto config_iterator = info_.hardware_parameters.find("motor_config");
  if (config_iterator == info_.hardware_parameters.end() || config_iterator->second.empty()) {
    error = "ros2_control hardware parameter 'motor_config' is required";
    return false;
  }
  if (!load_robot_config(config_iterator->second, config_, error)) {
    return false;
  }

  if (!parse_parameter(info_.hardware_parameters, "can_interface", config_.bus.interface_name, error) ||
    !parse_parameter(info_.hardware_parameters, "bitrate", config_.bus.bitrate, error) ||
    !parse_parameter(info_.hardware_parameters, "read_only", config_.read_only, error) ||
    !parse_parameter(info_.hardware_parameters, "poll_rate_hz", config_.poll_rate_hz, error))
  {
    return false;
  }

  int host_id = static_cast<int>(config_.bus.host_id);
  if (!parse_parameter(info_.hardware_parameters, "host_id", host_id, error)) {
    return false;
  }
  if (host_id < 0 || host_id > 255) {
    error = "hardware parameter 'host_id' must be in the range 0..255";
    return false;
  }
  config_.bus.host_id = static_cast<uint8_t>(host_id);

  int timeout_ms = static_cast<int>(config_.bus.response_timeout.count());
  if (!parse_parameter(info_.hardware_parameters, "response_timeout_ms", timeout_ms, error)) {
    return false;
  }
  config_.bus.response_timeout = std::chrono::milliseconds(timeout_ms);
  return validate_robot_config(config_, error);
}

bool RobstrideSystem::validate_hardware_joints(std::string & error)
{
  if (info_.joints.empty()) {
    error = "ros2_control hardware must declare at least one joint";
    return false;
  }
  if (info_.joints.size() != config_.motors.size()) {
    error = "ros2_control joint count does not match motor config count";
    return false;
  }

  std::vector<MotorConfig> ordered_motors;
  ordered_motors.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    const auto motor = std::find_if(
      config_.motors.begin(), config_.motors.end(),
      [&joint](const MotorConfig & candidate) { return candidate.joint_name == joint.name; });
    if (motor == config_.motors.end()) {
      error = "no motor config exists for ros2_control joint '" + joint.name + "'";
      return false;
    }
    ordered_motors.push_back(*motor);
    if (!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT))
    {
      error = "joint '" + joint.name + "' must export position, velocity, and effort states";
      return false;
    }
    if (!has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION)) {
      error = "joint '" + joint.name + "' must export a position command";
      return false;
    }
  }
  config_.motors = std::move(ordered_motors);
  return true;
}

std::vector<hardware_interface::StateInterface> RobstrideSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 3);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const std::string & name = info_.joints[index].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &state_positions_[index]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state_velocities_[index]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &state_efforts_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobstrideSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &command_positions_[index]);
  }
  return interfaces;
}

bool RobstrideSystem::read_all_initial_states(std::string & error)
{
  for (std::size_t index = 0; index < config_.motors.size(); ++index) {
    MotorState state;
    if (!bus_->read_encoder(config_.motors[index].id, state, error)) {
      return false;
    }
    const MotorConfig & motor = config_.motors[index];
    const double position = to_joint_position(motor, state.position_rad);
    const double velocity = to_joint_velocity(motor, state.velocity_rad_s);
    const double effort = to_joint_effort(motor, state.torque_nm);
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_states_[index] = state;
    state_positions_[index] = position;
    state_velocities_[index] = velocity;
    state_efforts_[index] = effort;
    command_positions_[index] = position;
    pending_commands_[index] = position;
  }
  return true;
}

hardware_interface::CallbackReturn RobstrideSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!initialized_ || !bus_) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string error;
  if (!bus_->connect(error)) {
    std::cerr << "[rs_control] " << error << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!config_.read_only) {
    if (!read_all_initial_states(error)) {
      std::cerr << "[rs_control] " << error << std::endl;
      bus_->disconnect();
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto & motor : config_.motors) {
      MotorState state;
      if (!bus_->disable(motor.id, state, error) ||
        !bus_->set_run_mode(motor.id, 0, error) ||
        !bus_->enable(motor.id, state, error))
      {
        std::cerr << "[rs_control] failed to activate " << motor.joint_name << ": " << error
                  << std::endl;
        disable_all_motors();
        bus_->disconnect();
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  io_error_ = false;
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    io_error_message_.clear();
  }
  active_ = true;
  start_io_thread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  stop_io_thread();
  if (bus_ && bus_->is_connected()) {
    disable_all_motors();
    bus_->disconnect();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobstrideSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (io_error_) {
    return hardware_interface::return_type::ERROR;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  for (std::size_t index = 0; index < latest_states_.size(); ++index) {
    if (latest_states_[index].valid) {
      state_positions_[index] = to_joint_position(
        config_.motors[index], latest_states_[index].position_rad);
      state_velocities_[index] = to_joint_velocity(
        config_.motors[index], latest_states_[index].velocity_rad_s);
      state_efforts_[index] = to_joint_effort(
        config_.motors[index], latest_states_[index].torque_nm);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (config_.read_only || !active_) {
    return hardware_interface::return_type::OK;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_commands_ = command_positions_;
  return io_error_ ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

void RobstrideSystem::disable_all_motors()
{
  if (!bus_ || !bus_->is_connected() || config_.read_only) {
    return;
  }
  for (const auto & motor : config_.motors) {
    MotorState state;
    std::string ignored_error;
    bus_->disable(motor.id, state, ignored_error);
  }
}

void RobstrideSystem::start_io_thread()
{
  stop_io_thread();
  stop_io_ = false;
  io_thread_ = std::thread(&RobstrideSystem::io_loop, this);
}

void RobstrideSystem::stop_io_thread()
{
  stop_io_ = true;
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void RobstrideSystem::report_io_error(const std::string & error)
{
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    io_error_message_ = error;
  }
  io_error_ = true;
  std::cerr << "[rs_control] CAN I/O error: " << error << std::endl;
}

void RobstrideSystem::io_loop()
{
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / config_.poll_rate_hz));
  auto next_cycle = std::chrono::steady_clock::now();

  while (!stop_io_) {
    for (std::size_t index = 0; index < config_.motors.size() && !stop_io_; ++index) {
      const MotorConfig & motor = config_.motors[index];
      MotorState state;
      std::string error;
      bool success = false;
      if (config_.read_only) {
        success = bus_->read_encoder(motor.id, state, error);
      } else {
        double command = 0.0;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          command = pending_commands_[index];
        }
        OperationCommand operation;
        operation.position_rad = to_motor_position(motor, command);
        operation.velocity_rad_s = 0.0;
        operation.torque_nm = 0.0;
        operation.kp = motor.kp;
        operation.kd = motor.kd;
        success = bus_->send_operation_command(motor.id, motor.model, operation, state, error);
      }

      if (!success) {
        disable_all_motors();
        report_io_error(error);
        stop_io_ = true;
        return;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_states_[index] = state;
      }
    }

    next_cycle += period;
    const auto now = std::chrono::steady_clock::now();
    if (next_cycle < now) {
      next_cycle = now;
    }
    std::this_thread::sleep_until(next_cycle);
  }
}

}  // namespace rs_control

PLUGINLIB_EXPORT_CLASS(rs_control::RobstrideSystem, hardware_interface::SystemInterface)
