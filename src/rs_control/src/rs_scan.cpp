#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rs_control/robstride_bus.hpp"

namespace
{

void print_usage(const char * executable)
{
  std::cout
    << "Usage: " << executable << " [--interface can0] [--timeout-ms 20]"
    << " [--start-id 1] [--end-id 255] [--bitrate 1000000]\n";
}

uint8_t parse_device_id(const std::string & value, const char * option)
{
  const int parsed = std::stoi(value);
  if (parsed < 1 || parsed > 255) {
    throw std::invalid_argument(std::string(option) + " must be in the range 1..255");
  }
  return static_cast<uint8_t>(parsed);
}

bool read_option(
  int & index, int argc, char ** argv, const std::string & option, std::string & value)
{
  if (std::string(argv[index]) != option || index + 1 >= argc) {
    return false;
  }
  value = argv[++index];
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rs_control::BusConfig bus_config;
  uint8_t first_id = 1;
  uint8_t last_id = 255;

  try {
    for (int index = 1; index < argc; ++index) {
      const std::string argument(argv[index]);
      if (argument == "--help" || argument == "-h") {
        print_usage(argv[0]);
        return 0;
      }
      std::string value;
      if (read_option(index, argc, argv, "--interface", value)) {
        bus_config.interface_name = value;
      } else if (read_option(index, argc, argv, "--timeout-ms", value)) {
        const int timeout_ms = std::stoi(value);
        if (timeout_ms <= 0) {
          throw std::invalid_argument("--timeout-ms must be positive");
        }
        bus_config.response_timeout = std::chrono::milliseconds(timeout_ms);
      } else if (read_option(index, argc, argv, "--start-id", value)) {
        first_id = parse_device_id(value, "--start-id");
      } else if (read_option(index, argc, argv, "--end-id", value)) {
        last_id = parse_device_id(value, "--end-id");
      } else if (argument == "--bitrate" && index + 1 < argc) {
        bus_config.bitrate = static_cast<uint32_t>(std::stoul(argv[++index]));
        if (bus_config.bitrate == 0) {
          throw std::invalid_argument("--bitrate must be positive");
        }
      } else {
        std::cerr << "Unknown or incomplete option: " << argument << "\n";
        print_usage(argv[0]);
        return 2;
      }
    }
  } catch (const std::exception & exception) {
    std::cerr << "Invalid scan option: " << exception.what() << "\n";
    return 2;
  }

  rs_control::RobstrideBus bus(bus_config);
  std::string error;
  if (!bus.connect(error)) {
    std::cerr << "Unable to connect to CAN bus: " << error << "\n";
    return 1;
  }

  std::vector<rs_control::DiscoveredMotor> motors;
  if (!bus.scan(motors, first_id, last_id, error)) {
    std::cerr << "CAN scan failed: " << error << "\n";
    return 1;
  }

  std::cout << "Found " << motors.size() << " RobStride motor(s) on "
            << bus_config.interface_name << ":\n";
  for (const auto & motor : motors) {
    std::cout << "  id=" << static_cast<int>(motor.id) << " uuid=0x" << std::hex
              << std::setfill('0');
    for (const uint8_t byte : motor.uuid) {
      std::cout << std::setw(2) << static_cast<int>(byte);
    }
    std::cout << std::dec << "\n";
  }
  return 0;
}
