#ifndef RS_CONTROL__ROBSTRIDE_SYSTEM_HPP_
#define RS_CONTROL__ROBSTRIDE_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rs_control/motor_config.hpp"

namespace rs_control
{

class RobstrideSystem final : public hardware_interface::SystemInterface
{
public:
  RobstrideSystem() = default;
  ~RobstrideSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool load_hardware_config(std::string & error);
  bool validate_hardware_joints(std::string & error);
  bool read_all_initial_states(std::string & error);
  void disable_all_motors();
  void start_io_thread();
  void stop_io_thread();
  void io_loop();
  void report_io_error(const std::string & error);

  RobotConfig config_;
  std::unique_ptr<RobstrideBus> bus_;

  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;
  std::vector<double> state_efforts_;
  std::vector<double> state_temperatures_;
  std::vector<double> command_positions_;
  std::vector<double> pending_commands_;
  std::vector<MotorState> latest_states_;

  mutable std::mutex data_mutex_;
  std::mutex error_mutex_;
  std::string io_error_message_;
  std::thread io_thread_;
  std::atomic<bool> stop_io_{true};
  std::atomic<bool> io_error_{false};
  bool initialized_{false};
  bool active_{false};
};

}  // namespace rs_control

#endif  // RS_CONTROL__ROBSTRIDE_SYSTEM_HPP_
