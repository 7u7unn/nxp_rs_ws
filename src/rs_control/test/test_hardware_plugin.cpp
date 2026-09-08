#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rs_control/robstride_system.hpp"

namespace
{

hardware_interface::InterfaceInfo interface_info(const std::string & name)
{
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::HardwareInfo make_hardware_info(const std::string & config_path)
{
  hardware_interface::HardwareInfo info;
  info.name = "robstride_system";
  info.type = "system";
  info.hardware_parameters["motor_config"] = config_path;
  info.hardware_parameters["read_only"] = "true";

  const std::vector<std::string> joints{
    "joint-1", "joint-2", "joint-3", "joint-4", "joint-5", "joint-6",
    "joint_right-finger"};
  for (const auto & name : joints) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.command_interfaces.push_back(interface_info(hardware_interface::HW_IF_POSITION));
    joint.state_interfaces.push_back(interface_info(hardware_interface::HW_IF_POSITION));
    joint.state_interfaces.push_back(interface_info(hardware_interface::HW_IF_VELOCITY));
    joint.state_interfaces.push_back(interface_info(hardware_interface::HW_IF_EFFORT));
    info.joints.push_back(joint);
  }
  return info;
}

std::string write_valid_config()
{
  const std::string path = "/tmp/rs_control_test_robstride.yaml";
  std::ofstream file(path);
  file << "can_interface: can0\n"
       << "bitrate: 1000000\n"
       << "host_id: 255\n"
       << "response_timeout_ms: 20\n"
       << "poll_rate_hz: 20.0\n"
       << "read_only: true\n"
       << "motors:\n";
  for (int index = 1; index <= 7; ++index) {
    file << "  - joint: " << (index == 7 ? "joint_right-finger" : "joint-" + std::to_string(index))
         << "\n"
         << "    id: " << index << "\n"
         << "    model: " << (index >= 5 ? "rs-05" : "rs-00") << "\n"
         << "    direction: 1\n"
         << "    position_scale: 1.0\n"
         << "    kp: 10.0\n"
         << "    kd: 1.0\n";
  }
  return path;
}

}  // namespace

TEST(RobstrideSystemTest, InitializesAndExportsConfiguredInterfaces)
{
  rs_control::RobstrideSystem system;
  const auto info = make_hardware_info(write_valid_config());

  EXPECT_EQ(
    system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_state_interfaces().size(), 21U);
  EXPECT_EQ(system.export_command_interfaces().size(), 7U);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
}

