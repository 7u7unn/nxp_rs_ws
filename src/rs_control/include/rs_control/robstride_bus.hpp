#ifndef RS_CONTROL__ROBSTRIDE_BUS_HPP_
#define RS_CONTROL__ROBSTRIDE_BUS_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rs_control
{

enum class MotorModel
{
  RS00,
  RS05,
};

struct MotorLimits
{
  double position_rad;
  double velocity_rad_s;
  double torque_nm;
  double kp;
  double kd;
};

const MotorLimits & limits_for_model(MotorModel model);

std::optional<MotorModel> parse_motor_model(const std::string & model);

std::string motor_model_name(MotorModel model);

struct MotorConfig
{
  std::string joint_name;
  uint8_t id{0};
  MotorModel model{MotorModel::RS00};
  int direction{1};
  double position_offset{0.0};
  double position_scale{1.0};
  double kp{10.0};
  double kd{1.0};
};

struct BusConfig
{
  std::string interface_name{"can0"};
  uint32_t bitrate{1000000};
  uint8_t host_id{0xFF};
  std::chrono::milliseconds response_timeout{20};
};

struct MotorState
{
  bool valid{false};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  double temperature_c{0.0};
  uint16_t status_flags{0};
};

struct OperationCommand
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  double kp{0.0};
  double kd{0.0};
};

struct DiscoveredMotor
{
  uint8_t id{0};
  std::array<uint8_t, 8> uuid{};
};

constexpr uint16_t kModeParameter = 0x7005;
constexpr uint16_t kMeasuredPositionParameter = 0x3016;
constexpr uint16_t kMeasuredVelocityParameter = 0x3017;
constexpr uint16_t kMeasuredTorqueParameter = 0x302C;
constexpr uint16_t kMechanicalPositionParameter = 0x7019;
constexpr uint16_t kMechanicalVelocityParameter = 0x701B;

uint32_t compose_extended_id(uint8_t communication_type, uint16_t extra_data, uint8_t device_id);

uint16_t encode_u16(double value, double minimum, double maximum);

double decode_u16(uint16_t value, double minimum, double maximum);

class RobstrideBus
{
public:
  explicit RobstrideBus(BusConfig config);
  ~RobstrideBus();

  RobstrideBus(const RobstrideBus &) = delete;
  RobstrideBus & operator=(const RobstrideBus &) = delete;

  bool connect(std::string & error);
  void disconnect();
  bool is_connected() const;

  bool ping(uint8_t device_id, DiscoveredMotor & discovered, std::string & error);
  bool scan(
    std::vector<DiscoveredMotor> & discovered,
    uint8_t first_id,
    uint8_t last_id,
    std::string & error);

  bool read_parameter(
    uint8_t device_id, uint16_t parameter, float & value, std::string & error);

  bool read_encoder(uint8_t device_id, MotorState & state, std::string & error);

  bool set_run_mode(uint8_t device_id, uint8_t mode, std::string & error);
  bool enable(uint8_t device_id, MotorState & state, std::string & error);
  bool disable(uint8_t device_id, MotorState & state, std::string & error);

  bool send_operation_command(
    uint8_t device_id,
    MotorModel model,
    const OperationCommand & command,
    MotorState & state,
    std::string & error);

private:
  struct ReceivedFrame
  {
    uint8_t communication_type{0};
    uint16_t extra_data{0};
    uint8_t source_id{0};
    std::array<uint8_t, 8> data{};
    uint8_t length{0};
  };

  bool send_frame(
    uint8_t communication_type,
    uint16_t extra_data,
    uint8_t device_id,
    const uint8_t * data,
    uint8_t length,
    std::string & error);

  bool receive_frame(
    const std::vector<uint8_t> & expected_types,
    uint8_t device_id,
    bool check_device_id,
    ReceivedFrame & frame,
    std::string & error);

  bool receive_status(
    uint8_t device_id,
    MotorState & state,
    std::string & error,
    MotorModel model = MotorModel::RS00);

  BusConfig config_;
  int socket_fd_{-1};
};

}  // namespace rs_control

#endif  // RS_CONTROL__ROBSTRIDE_BUS_HPP_
