#include "rs_control/robstride_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rs_control
{
namespace
{

constexpr uint8_t kGetDeviceId = 0;
constexpr uint8_t kOperationControl = 1;
constexpr uint8_t kOperationStatus = 2;
constexpr uint8_t kEnable = 3;
constexpr uint8_t kDisable = 4;
constexpr uint8_t kReadParameter = 17;
constexpr uint8_t kWriteParameter = 18;
constexpr uint8_t kFaultReport = 21;

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

uint16_t read_u16_be(const uint8_t * data)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

uint32_t read_u32_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
}

std::string hex_value(uint32_t value, unsigned int width)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setw(static_cast<int>(width)) << std::setfill('0') << value;
  return stream.str();
}

struct NamedBit
{
  uint32_t mask;
  const char * name;
};

template<std::size_t N>
std::string describe_bits(uint32_t value, const std::array<NamedBit, N> & names)
{
  std::ostringstream stream;
  bool first = true;
  uint32_t known_bits = 0;
  for (const auto & bit : names) {
    known_bits |= bit.mask;
    if ((value & bit.mask) != 0U) {
      if (!first) {
        stream << ", ";
      }
      stream << bit.name;
      first = false;
    }
  }
  const uint32_t unknown_bits = value & ~known_bits;
  if (unknown_bits != 0U) {
    if (!first) {
      stream << ", ";
    }
    stream << "unknown=" << hex_value(unknown_bits, 8);
    first = false;
  }
  if (first) {
    stream << "none";
  }
  return stream.str();
}

float read_float_le(const uint8_t * data)
{
  uint32_t bits = static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void write_u16_le(uint8_t * data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFFU);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

void write_u16_be(uint8_t * data, uint16_t value)
{
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  data[1] = static_cast<uint8_t>(value & 0xFFU);
}

}  // namespace

const MotorLimits & limits_for_model(MotorModel model)
{
  static const MotorLimits rs00{4.0 * M_PI, 50.0, 17.0, 500.0, 5.0};
  static const MotorLimits rs05{4.0 * M_PI, 33.0, 17.0, 500.0, 5.0};
  return model == MotorModel::RS05 ? rs05 : rs00;
}

std::optional<MotorModel> parse_motor_model(const std::string & model)
{
  if (model == "rs-00" || model == "RS-00" || model == "rs00" || model == "RS00") {
    return MotorModel::RS00;
  }
  if (model == "rs-05" || model == "RS-05" || model == "rs05" || model == "RS05") {
    return MotorModel::RS05;
  }
  return std::nullopt;
}

std::string motor_model_name(MotorModel model)
{
  return model == MotorModel::RS05 ? "rs-05" : "rs-00";
}

uint32_t compose_extended_id(uint8_t communication_type, uint16_t extra_data, uint8_t device_id)
{
  return (static_cast<uint32_t>(communication_type & 0x1FU) << 24) |
    (static_cast<uint32_t>(extra_data) << 8) | device_id;
}

uint16_t encode_u16(double value, double minimum, double maximum)
{
  if (maximum <= minimum || !std::isfinite(value)) {
    return 0;
  }
  const double clamped = std::clamp(value, minimum, maximum);
  const double normalized = (clamped - minimum) / (maximum - minimum);
  return static_cast<uint16_t>(std::llround(normalized * 65535.0));
}

double decode_u16(uint16_t value, double minimum, double maximum)
{
  if (maximum <= minimum) {
    return minimum;
  }
  return static_cast<double>(value) * (maximum - minimum) / 65535.0 + minimum;
}

std::string describe_status_flags(uint16_t status_flags)
{
  // Type-2 feedback stores these six protection flags in CAN-ID bits 13..8.
  static constexpr std::array<NamedBit, 6> names{{
    {0x2000U, "encoder uncalibrated"},
    {0x1000U, "stall/overload"},
    {0x0800U, "magnetic encoder fault"},
    {0x0400U, "overtemperature"},
    {0x0200U, "overcurrent"},
    {0x0100U, "undervoltage"},
  }};
  return hex_value(status_flags, 4) + " (" + describe_bits(status_flags, names) + ")";
}

std::string describe_fault_report(uint32_t fault_code, uint32_t warning_code)
{
  // Type-21 payload values are little-endian.  These names are the bits
  // documented by the RobStride RS0x protocol; unknown bits are preserved in
  // the output rather than silently discarded for newer firmware.
  static constexpr std::array<NamedBit, 11> fault_names{{
    {1U << 0, "motor overtemperature"},
    {1U << 1, "driver IC fault"},
    {1U << 2, "undervoltage"},
    {1U << 3, "overvoltage"},
    {1U << 4, "B-phase overcurrent"},
    {1U << 5, "C-phase overcurrent"},
    {1U << 7, "encoder uncalibrated"},
    {1U << 8, "hardware ID fault"},
    {1U << 9, "position initialization fault"},
    {1U << 14, "stall/overload"},
    {1U << 16, "A-phase overcurrent"},
  }};
  static constexpr std::array<NamedBit, 1> warning_names{{
    {1U << 0, "motor overtemperature warning"},
  }};
  return "fault=" + hex_value(fault_code, 8) + " (" +
    describe_bits(fault_code, fault_names) + "), warning=" +
    hex_value(warning_code, 8) + " (" + describe_bits(warning_code, warning_names) + ")";
}

ProtocolFrame make_operation_frame(
  uint8_t device_id, MotorModel model, const OperationCommand & command)
{
  const MotorLimits & limits = limits_for_model(model);
  const uint16_t torque = encode_u16(command.torque_nm, -limits.torque_nm, limits.torque_nm);
  const uint16_t position = encode_u16(
    command.position_rad, -limits.position_rad, limits.position_rad);
  const uint16_t velocity = encode_u16(
    command.velocity_rad_s, -limits.velocity_rad_s, limits.velocity_rad_s);
  const uint16_t kp = encode_u16(command.kp, 0.0, limits.kp);
  const uint16_t kd = encode_u16(command.kd, 0.0, limits.kd);

  ProtocolFrame frame;
  frame.id = compose_extended_id(kOperationControl, torque, device_id);
  write_u16_be(frame.data.data(), position);
  write_u16_be(frame.data.data() + 2, velocity);
  write_u16_be(frame.data.data() + 4, kp);
  write_u16_be(frame.data.data() + 6, kd);
  return frame;
}

ProtocolFrame make_feedback_request_frame(uint8_t host_id, uint8_t device_id)
{
  ProtocolFrame frame;
  frame.id = compose_extended_id(kOperationStatus, host_id, device_id);
  return frame;
}

ProtocolFrame make_parameter_read_frame(
  uint8_t host_id, uint8_t device_id, uint16_t parameter)
{
  ProtocolFrame frame;
  frame.id = compose_extended_id(kReadParameter, host_id, device_id);
  write_u16_le(frame.data.data(), parameter);
  return frame;
}

ProtocolFrame make_parameter_write_frame(
  uint8_t host_id, uint8_t device_id, uint16_t parameter,
  const std::array<uint8_t, 4> & value)
{
  ProtocolFrame frame;
  frame.id = compose_extended_id(kWriteParameter, host_id, device_id);
  write_u16_le(frame.data.data(), parameter);
  std::copy(value.cbegin(), value.cend(), frame.data.begin() + 4);
  return frame;
}

ProtocolFrame make_enable_frame(uint8_t host_id, uint8_t device_id)
{
  ProtocolFrame frame;
  frame.id = compose_extended_id(kEnable, host_id, device_id);
  return frame;
}

ProtocolFrame make_disable_frame(uint8_t host_id, uint8_t device_id)
{
  ProtocolFrame frame;
  frame.id = compose_extended_id(kDisable, host_id, device_id);
  return frame;
}

RobstrideBus::RobstrideBus(BusConfig config)
: config_(std::move(config))
{
}

RobstrideBus::~RobstrideBus()
{
  disconnect();
}

bool RobstrideBus::connect(std::string & error)
{
  if (is_connected()) {
    return true;
  }

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    error = errno_message("unable to create SocketCAN socket");
    return false;
  }

  ifreq interface_request{};
  std::strncpy(
    interface_request.ifr_name, config_.interface_name.c_str(),
    sizeof(interface_request.ifr_name) - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &interface_request) < 0) {
    error = errno_message("unable to resolve CAN interface '" + config_.interface_name + "'");
    disconnect();
    return false;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    error = errno_message("unable to bind CAN interface '" + config_.interface_name + "'");
    disconnect();
    return false;
  }

  const int current_flags = fcntl(socket_fd_, F_GETFL, 0);
  if (current_flags >= 0) {
    fcntl(socket_fd_, F_SETFL, current_flags | O_NONBLOCK);
  }

  can_filter filter{};
  filter.can_id = CAN_EFF_FLAG;
  filter.can_mask = CAN_EFF_FLAG;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    error = errno_message("unable to install SocketCAN filter");
    disconnect();
    return false;
  }

  return true;
}

void RobstrideBus::disconnect()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool RobstrideBus::is_connected() const
{
  return socket_fd_ >= 0;
}

bool RobstrideBus::send_frame(
  uint8_t communication_type,
  uint16_t extra_data,
  uint8_t device_id,
  const uint8_t * data,
  uint8_t length,
  std::string & error)
{
  ProtocolFrame frame;
  frame.id = compose_extended_id(communication_type, extra_data, device_id);
  frame.length = length;
  if (data != nullptr && length > 0) {
    std::copy(data, data + length, frame.data.begin());
  }
  return send_frame(frame, error);
}

bool RobstrideBus::send_frame(const ProtocolFrame & protocol_frame, std::string & error)
{
  if (!is_connected()) {
    error = "CAN bus is not connected";
    return false;
  }
  const uint8_t device_id = static_cast<uint8_t>(protocol_frame.id & 0xFFU);
  if (device_id == 0 || protocol_frame.length > CAN_MAX_DLEN) {
    error = "invalid RobStride frame target or data length";
    return false;
  }

  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG | (protocol_frame.id & CAN_EFF_MASK);
  frame.can_dlc = protocol_frame.length;
  if (protocol_frame.length > 0) {
    std::memcpy(frame.data, protocol_frame.data.data(), protocol_frame.length);
  }

  const ssize_t written = write(socket_fd_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    error = errno_message("failed to transmit RobStride CAN frame");
    return false;
  }
  return true;
}

bool RobstrideBus::receive_frame(
  const std::vector<uint8_t> & expected_types,
  uint8_t device_id,
  bool check_device_id,
  ReceivedFrame & frame,
  std::string & error)
{
  if (!is_connected()) {
    error = "CAN bus is not connected";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + config_.response_timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    pollfd descriptor{};
    descriptor.fd = socket_fd_;
    descriptor.events = POLLIN;
    const int poll_result = poll(&descriptor, 1, std::max<int>(1, remaining.count()));
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      error = errno_message("failed while waiting for RobStride CAN response");
      return false;
    }
    if (poll_result == 0) {
      break;
    }

    can_frame raw_frame{};
    const ssize_t received = read(socket_fd_, &raw_frame, sizeof(raw_frame));
    if (received != static_cast<ssize_t>(sizeof(raw_frame))) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      error = errno_message("failed to receive RobStride CAN frame");
      return false;
    }
    if ((raw_frame.can_id & CAN_EFF_FLAG) == 0) {
      continue;
    }

    const uint32_t can_id = raw_frame.can_id & CAN_EFF_MASK;
    const uint8_t communication_type = static_cast<uint8_t>((can_id >> 24) & 0x1FU);
    const uint16_t extra_data = static_cast<uint16_t>((can_id >> 8) & 0xFFFFU);
    const uint8_t source_id = static_cast<uint8_t>(can_id & 0xFFU);
    if (std::find(expected_types.begin(), expected_types.end(), communication_type) ==
      expected_types.end())
    {
      continue;
    }
    if (check_device_id) {
      // Type-2 and parameter responses put the responding motor ID in the
      // low byte of data-area 2.  Some firmware revisions use the low byte of
      // the CAN ID for type-21 fault reports instead; accept either documented
      // layout, but never accept a report that names neither this motor nor
      // the requested one.
      const bool matches_extra_id = (extra_data & 0xFFU) == device_id;
      const bool matches_source_id = source_id == device_id;
      const bool matches = communication_type == kFaultReport ?
        (matches_extra_id || matches_source_id) : matches_extra_id;
      if (!matches) {
        continue;
      }
    }

    frame.communication_type = communication_type;
    frame.extra_data = extra_data;
    frame.source_id = source_id;
    frame.length = std::min<uint8_t>(raw_frame.can_dlc, CAN_MAX_DLEN);
    std::copy(raw_frame.data, raw_frame.data + frame.length, frame.data.begin());
    return true;
  }

  std::ostringstream message;
  message << "timeout waiting for RobStride response from motor " << static_cast<int>(device_id);
  error = message.str();
  return false;
}

bool RobstrideBus::receive_status(
  uint8_t device_id, MotorState & state, std::string & error, MotorModel model)
{
  state.valid = false;
  state.status_flags = 0;
  state.fault_code = 0;
  state.warning_code = 0;
  ReceivedFrame frame;
  if (!receive_frame({kOperationStatus, kFaultReport}, device_id, true, frame, error)) {
    return false;
  }

  if (frame.communication_type == kFaultReport) {
    state.status_flags = 0;
    if (frame.length < 8) {
      error = "motor " + std::to_string(device_id) +
        " returned a truncated fault report (length=" +
        std::to_string(frame.length) + ", data=";
      for (std::size_t index = 0; index < frame.length; ++index) {
        if (index != 0) {
          error += ":";
        }
        error += hex_value(frame.data[index], 2).substr(2);
      }
      error += ")";
      return false;
    }
    state.fault_code = read_u32_le(frame.data.data());
    state.warning_code = read_u32_le(frame.data.data() + 4);
    error = "motor " + std::to_string(device_id) + " returned a fault report (" +
      describe_fault_report(state.fault_code, state.warning_code) +
      ", extra_data=" + hex_value(frame.extra_data, 4) +
      ", source_id=" + hex_value(frame.source_id, 2) +
      ", data=";
    for (std::size_t index = 0; index < frame.length; ++index) {
      if (index != 0) {
        error += ":";
      }
      error += hex_value(frame.data[index], 2).substr(2);
    }
    error += ")";
    return false;
  }

  state.status_flags = static_cast<uint16_t>(frame.extra_data & 0x3F00U);
  if (state.status_flags != 0U) {
    error = "motor " + std::to_string(device_id) +
      " operation status reports protection flags " + describe_status_flags(state.status_flags) +
      " (extra_data=" + hex_value(frame.extra_data, 4) + ")";
    return false;
  }
  if (frame.length < 8) {
    error = "operation status frame is shorter than eight bytes";
    return false;
  }

  const MotorLimits & limits = limits_for_model(model);
  const uint16_t position = read_u16_be(frame.data.data());
  const uint16_t velocity = read_u16_be(frame.data.data() + 2);
  const uint16_t torque = read_u16_be(frame.data.data() + 4);
  const uint16_t temperature = read_u16_be(frame.data.data() + 6);
  state.position_rad = decode_u16(position, -limits.position_rad, limits.position_rad);
  state.velocity_rad_s = decode_u16(velocity, -limits.velocity_rad_s, limits.velocity_rad_s);
  state.torque_nm = decode_u16(torque, -limits.torque_nm, limits.torque_nm);
  state.temperature_c = static_cast<double>(temperature) * 0.1;
  state.valid = true;
  return true;
}

bool RobstrideBus::ping(uint8_t device_id, DiscoveredMotor & discovered, std::string & error)
{
  const uint8_t empty_data[8]{};
  if (!send_frame(kGetDeviceId, config_.host_id, device_id, empty_data, 8, error)) {
    return false;
  }

  ReceivedFrame frame;
  if (!receive_frame({kGetDeviceId}, device_id, false, frame, error)) {
    return false;
  }

  discovered.id = device_id;
  discovered.uuid.fill(0);
  std::copy(frame.data.begin(), frame.data.begin() + std::min<uint8_t>(frame.length, 8),
    discovered.uuid.begin());
  return true;
}

bool RobstrideBus::scan(
  std::vector<DiscoveredMotor> & discovered,
  uint8_t first_id,
  uint8_t last_id,
  std::string & error)
{
  if (first_id == 0 || last_id < first_id) {
    error = "invalid scan ID range";
    return false;
  }
  discovered.clear();
  for (unsigned int id = first_id; id <= last_id; ++id) {
    DiscoveredMotor motor;
    std::string ping_error;
    if (ping(static_cast<uint8_t>(id), motor, ping_error)) {
      discovered.push_back(motor);
    } else if (ping_error.rfind("timeout", 0) != 0) {
      error = ping_error;
      return false;
    }
  }
  return true;
}

bool RobstrideBus::read_parameter(
  uint8_t device_id, uint16_t parameter, float & value, std::string & error)
{
  const ProtocolFrame request = make_parameter_read_frame(config_.host_id, device_id, parameter);
  if (!send_frame(request, error)) {
    return false;
  }

  ReceivedFrame frame;
  if (!receive_frame({kReadParameter}, device_id, false, frame, error)) {
    return false;
  }
  if (frame.length < 8) {
    error = "parameter response is shorter than eight bytes";
    return false;
  }

  value = read_float_le(frame.data.data() + 4);
  if (!std::isfinite(value)) {
    error = "parameter response contained a non-finite value";
    return false;
  }
  return true;
}

bool RobstrideBus::read_status(
  uint8_t device_id, MotorModel model, MotorState & state, std::string & error)
{
  const ProtocolFrame request = make_feedback_request_frame(config_.host_id, device_id);
  if (!send_frame(request, error)) {
    return false;
  }
  return receive_status(device_id, state, error, model);
}

bool RobstrideBus::read_encoder(uint8_t device_id, MotorState & state, std::string & error)
{
  float position = 0.0F;
  float velocity = 0.0F;
  float torque = 0.0F;
  if (!read_parameter(device_id, kMechanicalPositionParameter, position, error) ||
    !read_parameter(device_id, kMechanicalVelocityParameter, velocity, error) ||
    !read_parameter(device_id, kMeasuredTorqueParameter, torque, error))
  {
    state.valid = false;
    return false;
  }
  state.position_rad = position;
  state.velocity_rad_s = velocity;
  state.torque_nm = torque;
  state.valid = true;
  return true;
}

bool RobstrideBus::set_run_mode(uint8_t device_id, uint8_t mode, std::string & error)
{
  if (mode != 0 && mode != 1 && mode != 2 && mode != 3 && mode != 5) {
    error = "unsupported RobStride run mode";
    return false;
  }

  const std::array<uint8_t, 4> value{mode, 0, 0, 0};
  const ProtocolFrame request = make_parameter_write_frame(
    config_.host_id, device_id, kModeParameter, value);
  if (!send_frame(request, error)) {
    return false;
  }
  MotorState state;
  return receive_status(device_id, state, error);
}

bool RobstrideBus::enable(uint8_t device_id, MotorState & state, std::string & error)
{
  const ProtocolFrame request = make_enable_frame(config_.host_id, device_id);
  if (!send_frame(request, error)) {
    return false;
  }
  return receive_status(device_id, state, error);
}

bool RobstrideBus::disable(uint8_t device_id, MotorState & state, std::string & error)
{
  const ProtocolFrame request = make_disable_frame(config_.host_id, device_id);
  if (!send_frame(request, error)) {
    return false;
  }
  return receive_status(device_id, state, error);
}

bool RobstrideBus::send_operation_command(
  uint8_t device_id,
  MotorModel model,
  const OperationCommand & command,
  MotorState & state,
  std::string & error)
{
  const ProtocolFrame request = make_operation_frame(device_id, model, command);
  if (!send_frame(request, error)) {
    return false;
  }
  if (!receive_status(device_id, state, error, model)) {
    return false;
  }
  return true;
}

}  // namespace rs_control
