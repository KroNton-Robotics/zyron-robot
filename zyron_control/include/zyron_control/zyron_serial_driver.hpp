#ifndef ZYRON_CONTROL__ZYRON_SERIAL_DRIVER_HPP_
#define ZYRON_CONTROL__ZYRON_SERIAL_DRIVER_HPP_

#include <libserial/SerialPort.h>
#include <realtime_tools/realtime_buffer.hpp>
#include <zyron_protocol/binary_protocol.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace zyron_control
{

struct DriverConfig
{
  std::string port{"/dev/ttyUSB0"};
  int baud_rate{460800};
  float left_counts_per_rev{228.0F};
  float right_counts_per_rev{224.0F};
  float max_speed_rad_s{20.94F};
  float start_threshold_rad_s{0.5F};
  float stop_threshold_rad_s{0.2F};
  int left_pwm_min{60};
  int right_pwm_min{60};
  int pwm_max{255};
  float left_kp{1.0F};
  float left_ki{2.0F};
  float right_kp{1.0F};
  float right_ki{2.0F};
  float max_acceleration_rad_s2{15.0F};
  float velocity_filter_tau_s{0.1F};
  std::chrono::milliseconds firmware_watchdog{200};
  std::chrono::milliseconds command_period{10};
  std::chrono::milliseconds startup_timeout{10000};
  std::chrono::milliseconds state_timeout{250};
  std::chrono::milliseconds imu_timeout{250};
};

enum class MotionInhibitReason : uint8_t
{
  NONE,
  DRIVER_STOPPED,
  HANDSHAKE,
  TELEMETRY_STALE,
  TELEMETRY_REARM,
  IMU_STALE,
  FIRMWARE_COMMAND_STALE,
};

struct DriverSample
{
  std::array<double, 2> wheel_position{{0.0, 0.0}};
  std::array<double, 2> wheel_velocity{{0.0, 0.0}};
  std::array<double, 4> imu_orientation{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> imu_angular_velocity{{0.0, 0.0, 0.0}};
  std::array<double, 3> imu_linear_acceleration{{0.0, 0.0, 0.0}};
  std::chrono::steady_clock::time_point wheel_stamp{};
  std::chrono::steady_clock::time_point imu_stamp{};
  std::uint64_t wheel_sequence{0};
  std::uint64_t imu_sequence{0};
  std::uint64_t telemetry_sequence_gaps{0};
  std::uint64_t host_crc_errors{0};
  std::uint64_t host_frame_errors{0};
  uint16_t firmware_status{0};
  uint16_t firmware_crc_errors{0};
  uint16_t firmware_frame_errors{0};
  int16_t left_applied_pwm{0};
  int16_t right_applied_pwm{0};
  bool wheel_valid{false};
  bool imu_valid{false};
  bool config_acknowledged{false};
  bool motion_allowed{false};
};

class ZyronSerialDriver
{
public:
  explicit ZyronSerialDriver(DriverConfig config);
  ~ZyronSerialDriver();

  ZyronSerialDriver(const ZyronSerialDriver &) = delete;
  ZyronSerialDriver & operator=(const ZyronSerialDriver &) = delete;

  bool start();
  void stop() noexcept;
  void request_stop() noexcept;
  void set_velocity_commands(double left_rad_s, double right_rad_s) noexcept;
  bool latest_sample(DriverSample & sample) noexcept;
  bool healthy() const noexcept;
  bool configured() const noexcept;
  MotionInhibitReason motion_inhibit_reason() const noexcept;
  const std::string & startup_error() const noexcept;

  static bool validate_config(const DriverConfig & config) noexcept;
  static bool wheel_state_is_fresh(
    const DriverSample & sample, std::chrono::milliseconds timeout,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) noexcept;
  static bool imu_state_is_fresh(
    const DriverSample & sample, std::chrono::milliseconds timeout,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) noexcept;
  static const char * inhibit_reason_text(MotionInhibitReason reason) noexcept;

private:
  static LibSerial::BaudRate baud_rate_from_int(int baud_rate) noexcept;
  zyron_protocol::ConfigPayload protocol_config() const noexcept;
  void io_loop() noexcept;
  void write_packet(
    zyron_protocol::MessageType type, const uint8_t * payload, std::size_t payload_size);
  void write_config();
  void write_command(float left_rad_s, float right_rad_s, bool enable);
  void close_port() noexcept;

  DriverConfig config_;
  LibSerial::SerialPort serial_port_;
  realtime_tools::RealtimeBuffer<DriverSample> sample_buffer_;
  std::thread io_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> healthy_{false};
  std::atomic<bool> configured_{false};
  std::atomic<bool> emergency_stop_{true};
  std::atomic<double> left_command_rad_s_{0.0};
  std::atomic<double> right_command_rad_s_{0.0};
  std::atomic<MotionInhibitReason> inhibit_reason_{MotionInhibitReason::DRIVER_STOPPED};
  std::atomic<uint16_t> transmit_sequence_{0};
  uint16_t config_crc_{0};
  std::string startup_error_;
};

}  // namespace zyron_control

#endif  // ZYRON_CONTROL__ZYRON_SERIAL_DRIVER_HPP_
