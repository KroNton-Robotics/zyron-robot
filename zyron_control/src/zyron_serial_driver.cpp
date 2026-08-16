#include "zyron_control/zyron_serial_driver.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

namespace zyron_control
{

ZyronSerialDriver::ZyronSerialDriver(DriverConfig config)
: config_(std::move(config)), sample_buffer_(DriverSample{})
{
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t payload_size = 0;
  const auto wire_config = protocol_config();
  if (zyron_protocol::encodeConfig(wire_config, payload, sizeof(payload), payload_size))
  {
    config_crc_ = zyron_protocol::crc16Ccitt(payload, payload_size);
  }
}

ZyronSerialDriver::~ZyronSerialDriver()
{
  stop();
}

bool ZyronSerialDriver::validate_config(const DriverConfig & c) noexcept
{
  return !c.port.empty() && baud_rate_from_int(c.baud_rate) != LibSerial::BaudRate::BAUD_INVALID &&
    std::isfinite(c.left_counts_per_rev) && c.left_counts_per_rev > 1.0F &&
    std::isfinite(c.right_counts_per_rev) && c.right_counts_per_rev > 1.0F &&
    std::isfinite(c.max_speed_rad_s) && c.max_speed_rad_s > 0.5F &&
    std::isfinite(c.start_threshold_rad_s) && std::isfinite(c.stop_threshold_rad_s) &&
    c.stop_threshold_rad_s >= 0.0F && c.start_threshold_rad_s > c.stop_threshold_rad_s &&
    c.start_threshold_rad_s < c.max_speed_rad_s && c.left_pwm_min >= 0 &&
    c.right_pwm_min >= 0 && c.left_pwm_min <= c.pwm_max && c.right_pwm_min <= c.pwm_max &&
    c.pwm_max > 0 && c.pwm_max <= 255 && std::isfinite(c.left_kp) && c.left_kp >= 0.0F &&
    std::isfinite(c.left_ki) && c.left_ki >= 0.0F && std::isfinite(c.right_kp) &&
    c.right_kp >= 0.0F && std::isfinite(c.right_ki) && c.right_ki >= 0.0F &&
    std::isfinite(c.max_acceleration_rad_s2) && c.max_acceleration_rad_s2 > 0.0F &&
    std::isfinite(c.velocity_filter_tau_s) && c.velocity_filter_tau_s >= 0.01F &&
    c.firmware_watchdog >= std::chrono::milliseconds(50) &&
    c.firmware_watchdog <= std::chrono::milliseconds(2000) &&
    c.command_period.count() > 0 && c.startup_timeout.count() > 0 &&
    c.state_timeout.count() > 0 && c.imu_timeout.count() > 0;
}

LibSerial::BaudRate ZyronSerialDriver::baud_rate_from_int(int baud_rate) noexcept
{
  switch (baud_rate)
  {
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    case 460800: return LibSerial::BaudRate::BAUD_460800;
    default: return LibSerial::BaudRate::BAUD_INVALID;
  }
}

zyron_protocol::ConfigPayload ZyronSerialDriver::protocol_config() const noexcept
{
  zyron_protocol::ConfigPayload result;
  result.left_counts_per_rev = config_.left_counts_per_rev;
  result.right_counts_per_rev = config_.right_counts_per_rev;
  result.max_speed_rad_s = config_.max_speed_rad_s;
  result.start_threshold_rad_s = config_.start_threshold_rad_s;
  result.stop_threshold_rad_s = config_.stop_threshold_rad_s;
  result.left_pwm_min = static_cast<uint8_t>(config_.left_pwm_min);
  result.right_pwm_min = static_cast<uint8_t>(config_.right_pwm_min);
  result.pwm_max = static_cast<uint8_t>(config_.pwm_max);
  result.left_kp = config_.left_kp;
  result.left_ki = config_.left_ki;
  result.right_kp = config_.right_kp;
  result.right_ki = config_.right_ki;
  result.max_acceleration_rad_s2 = config_.max_acceleration_rad_s2;
  result.velocity_filter_tau_s = config_.velocity_filter_tau_s;
  result.command_watchdog_ms = static_cast<uint16_t>(config_.firmware_watchdog.count());
  return result;
}

bool ZyronSerialDriver::start()
{
  startup_error_.clear();
  if (running_.load(std::memory_order_acquire))
  {
    startup_error_ = "driver is already running";
    return false;
  }
  if (!validate_config(config_))
  {
    startup_error_ = "driver configuration is invalid";
    return false;
  }
  if (io_thread_.joinable()) io_thread_.join();
  try
  {
    serial_port_.Open(config_.port);
    serial_port_.SetBaudRate(baud_rate_from_int(config_.baud_rate));
    serial_port_.FlushIOBuffers();
  }
  catch (const std::exception & exception)
  {
    startup_error_ = exception.what();
    close_port();
    return false;
  }
  catch (...)
  {
    startup_error_ = "unknown serial-port error";
    close_port();
    return false;
  }

  sample_buffer_.initRT(DriverSample{});
  transmit_sequence_.store(0, std::memory_order_relaxed);
  left_command_rad_s_.store(0.0, std::memory_order_relaxed);
  right_command_rad_s_.store(0.0, std::memory_order_relaxed);
  emergency_stop_.store(false, std::memory_order_release);
  configured_.store(false, std::memory_order_release);
  inhibit_reason_.store(MotionInhibitReason::HANDSHAKE, std::memory_order_release);
  healthy_.store(true, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  try
  {
    io_thread_ = std::thread(&ZyronSerialDriver::io_loop, this);
  }
  catch (const std::exception & exception)
  {
    startup_error_ = exception.what();
    running_.store(false, std::memory_order_release);
    healthy_.store(false, std::memory_order_release);
    close_port();
    return false;
  }
  return true;
}

void ZyronSerialDriver::request_stop() noexcept
{
  emergency_stop_.store(true, std::memory_order_release);
  left_command_rad_s_.store(0.0, std::memory_order_relaxed);
  right_command_rad_s_.store(0.0, std::memory_order_relaxed);
  inhibit_reason_.store(MotionInhibitReason::DRIVER_STOPPED, std::memory_order_release);
}

void ZyronSerialDriver::stop() noexcept
{
  request_stop();
  running_.store(false, std::memory_order_release);
  if (io_thread_.joinable()) io_thread_.join();
  close_port();
  configured_.store(false, std::memory_order_release);
  healthy_.store(false, std::memory_order_release);
}

void ZyronSerialDriver::set_velocity_commands(double left_rad_s, double right_rad_s) noexcept
{
  left_command_rad_s_.store(left_rad_s, std::memory_order_relaxed);
  right_command_rad_s_.store(right_rad_s, std::memory_order_relaxed);
}

bool ZyronSerialDriver::latest_sample(DriverSample & sample) noexcept
{
  const DriverSample * latest = sample_buffer_.readFromRT();
  if (latest == nullptr) return false;
  sample = *latest;
  return sample.wheel_valid;
}

bool ZyronSerialDriver::healthy() const noexcept {return healthy_.load(std::memory_order_acquire);}
bool ZyronSerialDriver::configured() const noexcept {return configured_.load(std::memory_order_acquire);}
MotionInhibitReason ZyronSerialDriver::motion_inhibit_reason() const noexcept
{return inhibit_reason_.load(std::memory_order_acquire);}
const std::string & ZyronSerialDriver::startup_error() const noexcept {return startup_error_;}

const char * ZyronSerialDriver::inhibit_reason_text(MotionInhibitReason reason) noexcept
{
  switch (reason)
  {
    case MotionInhibitReason::NONE: return "none";
    case MotionInhibitReason::DRIVER_STOPPED: return "driver stopped";
    case MotionInhibitReason::HANDSHAKE: return "firmware handshake incomplete";
    case MotionInhibitReason::TELEMETRY_STALE: return "telemetry stale";
    case MotionInhibitReason::TELEMETRY_REARM: return "waiting for three sequential telemetry packets";
    case MotionInhibitReason::IMU_STALE: return "firmware reports stale IMU";
    case MotionInhibitReason::FIRMWARE_COMMAND_STALE: return "firmware command watchdog active";
  }
  return "unknown";
}

bool ZyronSerialDriver::wheel_state_is_fresh(
  const DriverSample & sample, std::chrono::milliseconds timeout,
  std::chrono::steady_clock::time_point now) noexcept
{
  return sample.wheel_valid && timeout.count() > 0 && now >= sample.wheel_stamp &&
         now - sample.wheel_stamp <= timeout;
}

bool ZyronSerialDriver::imu_state_is_fresh(
  const DriverSample & sample, std::chrono::milliseconds timeout,
  std::chrono::steady_clock::time_point now) noexcept
{
  return sample.imu_valid && timeout.count() > 0 && now >= sample.imu_stamp &&
         now - sample.imu_stamp <= timeout;
}

void ZyronSerialDriver::write_packet(
  zyron_protocol::MessageType type, const uint8_t * payload, std::size_t payload_size)
{
  uint8_t encoded[zyron_protocol::kMaxEncodedFrame];
  const uint16_t sequence = transmit_sequence_.fetch_add(1, std::memory_order_relaxed);
  const std::size_t size = zyron_protocol::makePacket(
    type, sequence, payload, static_cast<uint16_t>(payload_size), encoded, sizeof(encoded));
  if (size != 0)
  {
    serial_port_.Write(std::string(reinterpret_cast<const char *>(encoded), size));
  }
}

void ZyronSerialDriver::write_config()
{
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t payload_size = 0;
  const auto config = protocol_config();
  if (zyron_protocol::encodeConfig(config, payload, sizeof(payload), payload_size))
  {
    write_packet(zyron_protocol::MessageType::CONFIG, payload, payload_size);
  }
}

void ZyronSerialDriver::write_command(float left_rad_s, float right_rad_s, bool enable)
{
  zyron_protocol::CommandPayload command;
  command.left_target_rad_s = std::isfinite(left_rad_s) ? left_rad_s : 0.0F;
  command.right_target_rad_s = std::isfinite(right_rad_s) ? right_rad_s : 0.0F;
  command.enable = enable ? 1 : 0;
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t payload_size = 0;
  if (zyron_protocol::encodeCommand(command, payload, sizeof(payload), payload_size))
  {
    write_packet(zyron_protocol::MessageType::COMMAND, payload, payload_size);
  }
}

void ZyronSerialDriver::io_loop() noexcept
{
  DriverSample sample;
  uint8_t encoded[zyron_protocol::kMaxEncodedFrame];
  std::size_t encoded_size = 0;
  bool discard_frame = false;
  bool hello_seen = false;
  bool have_telemetry_sequence = false;
  uint16_t previous_telemetry_sequence = 0;
  uint8_t telemetry_rearm_count = 0;
  std::uint64_t host_crc_errors = 0;
  std::uint64_t host_frame_errors = 0;
  auto next_command = std::chrono::steady_clock::now();
  auto next_config = next_command;

  try
  {
    while (running_.load(std::memory_order_acquire))
    {
      const int available = serial_port_.GetNumberOfBytesAvailable();
      if (available > 0)
      {
        std::string chunk;
        try
        {
          serial_port_.Read(chunk, static_cast<std::size_t>(std::min(available, 512)), 2);
        }
        catch (const LibSerial::ReadTimeout &) {}

        int frames_processed = 0;
        for (const unsigned char byte : chunk)
        {
          if (byte != 0)
          {
            if (!discard_frame && encoded_size < sizeof(encoded))
            {
              encoded[encoded_size++] = byte;
            }
            else if (!discard_frame)
            {
              discard_frame = true;
              encoded_size = 0;
              ++host_frame_errors;
            }
            continue;
          }
          if (discard_frame || encoded_size == 0)
          {
            discard_frame = false;
            encoded_size = 0;
            telemetry_rearm_count = 0;
            continue;
          }
          if (++frames_processed > 8) break;

          uint8_t decoded[zyron_protocol::kMaxDecodedFrame];
          zyron_protocol::PacketView packet;
          bool crc_error = false;
          const bool valid = zyron_protocol::parsePacket(
            encoded, encoded_size, decoded, packet, crc_error);
          encoded_size = 0;
          if (!valid)
          {
            if (crc_error) ++host_crc_errors;
            else ++host_frame_errors;
            telemetry_rearm_count = 0;
            continue;
          }

          if (packet.type == zyron_protocol::MessageType::HELLO)
          {
            zyron_protocol::HelloPayload hello;
            if (zyron_protocol::decodeHello(packet.payload, packet.payload_size, hello))
            {
              hello_seen = true;
              configured_.store(false, std::memory_order_release);
              sample.config_acknowledged = false;
              telemetry_rearm_count = 0;
              next_config = std::chrono::steady_clock::now();
            }
            continue;
          }
          if (packet.type == zyron_protocol::MessageType::CONFIG_ACK)
          {
            zyron_protocol::ConfigAckPayload ack;
            if (zyron_protocol::decodeConfigAck(packet.payload, packet.payload_size, ack) &&
              ack.config_crc == config_crc_ &&
              ack.result == static_cast<uint8_t>(zyron_protocol::ConfigResult::ACCEPTED))
            {
              configured_.store(true, std::memory_order_release);
              sample.config_acknowledged = true;
            }
            continue;
          }
          if (packet.type != zyron_protocol::MessageType::TELEMETRY ||
            !configured_.load(std::memory_order_acquire)) continue;

          zyron_protocol::TelemetryPayload telemetry;
          if (!zyron_protocol::decodeTelemetry(packet.payload, packet.payload_size, telemetry))
          {
            ++host_frame_errors;
            telemetry_rearm_count = 0;
            continue;
          }
          const std::array<float, 12> finite_values{{
            telemetry.left_velocity_rad_s, telemetry.right_velocity_rad_s,
            telemetry.qx, telemetry.qy, telemetry.qz, telemetry.qw,
            telemetry.ax, telemetry.ay, telemetry.az,
            telemetry.gx, telemetry.gy, telemetry.gz}};
          if (!std::all_of(finite_values.begin(), finite_values.end(),
              [](float value) {return std::isfinite(value);}))
          {
            ++host_frame_errors;
            telemetry_rearm_count = 0;
            continue;
          }

          const bool sequential = !have_telemetry_sequence ||
            packet.sequence == static_cast<uint16_t>(previous_telemetry_sequence + 1U);
          if (have_telemetry_sequence && !sequential) ++sample.telemetry_sequence_gaps;
          previous_telemetry_sequence = packet.sequence;
          have_telemetry_sequence = true;
          const bool firmware_configured =
            (telemetry.status_flags & zyron_protocol::STATUS_CONFIGURED) != 0;
          const bool imu_ready =
            (telemetry.status_flags & zyron_protocol::STATUS_IMU_READY) != 0 &&
            (telemetry.status_flags & zyron_protocol::STATUS_IMU_STALE) == 0;
          if (sequential && firmware_configured && imu_ready)
          {
            if (telemetry_rearm_count < 3) ++telemetry_rearm_count;
          }
          else telemetry_rearm_count = 0;

          const auto stamp = std::chrono::steady_clock::now();
          sample.wheel_position = {
            static_cast<double>(telemetry.left_encoder_count) * 2.0 * M_PI /
              static_cast<double>(config_.left_counts_per_rev),
            static_cast<double>(telemetry.right_encoder_count) * 2.0 * M_PI /
              static_cast<double>(config_.right_counts_per_rev)};
          sample.wheel_velocity = {
            telemetry.left_velocity_rad_s, telemetry.right_velocity_rad_s};
          sample.imu_orientation = {telemetry.qx, telemetry.qy, telemetry.qz, telemetry.qw};
          sample.imu_linear_acceleration = {telemetry.ax, telemetry.ay, telemetry.az};
          sample.imu_angular_velocity = {telemetry.gx, telemetry.gy, telemetry.gz};
          sample.left_applied_pwm = telemetry.left_applied_pwm;
          sample.right_applied_pwm = telemetry.right_applied_pwm;
          sample.firmware_status = telemetry.status_flags;
          sample.host_crc_errors = host_crc_errors;
          sample.host_frame_errors = host_frame_errors;
          sample.firmware_crc_errors = telemetry.rx_crc_errors;
          sample.firmware_frame_errors = telemetry.rx_frame_errors;
          sample.wheel_stamp = stamp;
          sample.wheel_valid = firmware_configured;
          ++sample.wheel_sequence;
          sample.imu_valid = imu_ready;
          if (imu_ready)
          {
            sample.imu_stamp = stamp;
            ++sample.imu_sequence;
          }
          sample.motion_allowed = telemetry_rearm_count >= 3;
          sample_buffer_.writeFromNonRT(sample);
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (hello_seen && !configured_.load(std::memory_order_acquire) && now >= next_config)
      {
        write_config();
        next_config = now + std::chrono::milliseconds(250);
      }
      if (now >= next_command)
      {
        MotionInhibitReason reason = MotionInhibitReason::NONE;
        if (!configured_.load(std::memory_order_acquire)) reason = MotionInhibitReason::HANDSHAKE;
        else if (!wheel_state_is_fresh(sample, config_.state_timeout, now))
        {
          reason = MotionInhibitReason::TELEMETRY_STALE;
          telemetry_rearm_count = 0;
          sample.motion_allowed = false;
        }
        else if (!sample.imu_valid) reason = MotionInhibitReason::IMU_STALE;
        else if (!sample.motion_allowed) reason = MotionInhibitReason::TELEMETRY_REARM;
        else if ((sample.firmware_status & zyron_protocol::STATUS_COMMAND_STALE) != 0)
          reason = MotionInhibitReason::FIRMWARE_COMMAND_STALE;
        if (emergency_stop_.load(std::memory_order_acquire))
          reason = MotionInhibitReason::DRIVER_STOPPED;
        inhibit_reason_.store(reason, std::memory_order_release);
        const bool enabled = reason == MotionInhibitReason::NONE;
        if (configured_.load(std::memory_order_acquire))
        {
          write_command(
            static_cast<float>(left_command_rad_s_.load(std::memory_order_relaxed)),
            static_cast<float>(right_command_rad_s_.load(std::memory_order_relaxed)), enabled);
        }
        next_command = now + config_.command_period;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (serial_port_.IsOpen()) write_command(0.0F, 0.0F, false);
  }
  catch (...)
  {
    healthy_.store(false, std::memory_order_release);
    running_.store(false, std::memory_order_release);
  }
  close_port();
}

void ZyronSerialDriver::close_port() noexcept
{
  if (!serial_port_.IsOpen()) return;
  try {serial_port_.Close();} catch (...) {}
}

}  // namespace zyron_control
