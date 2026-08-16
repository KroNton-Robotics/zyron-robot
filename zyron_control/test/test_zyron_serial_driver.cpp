#include "zyron_control/zyron_serial_driver.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace zyron_control
{
namespace
{

using namespace std::chrono_literals;

std::vector<uint8_t> make_frame(
  zyron_protocol::MessageType type, uint16_t sequence,
  const uint8_t * payload, std::size_t payload_size)
{
  std::vector<uint8_t> frame(zyron_protocol::kMaxEncodedFrame);
  const std::size_t size = zyron_protocol::makePacket(
    type, sequence, payload, static_cast<uint16_t>(payload_size), frame.data(), frame.size());
  frame.resize(size);
  return frame;
}

void write_all(int descriptor, const std::vector<uint8_t> & data)
{
  std::size_t offset = 0;
  while (offset < data.size())
  {
    const ssize_t count = write(descriptor, data.data() + offset, data.size() - offset);
    if (count > 0) offset += static_cast<std::size_t>(count);
    else std::this_thread::sleep_for(1ms);
  }
}

bool read_packet(
  int descriptor, zyron_protocol::PacketView & packet,
  std::array<uint8_t, zyron_protocol::kMaxDecodedFrame> & decoded,
  std::chrono::milliseconds timeout)
{
  std::vector<uint8_t> encoded;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    uint8_t buffer[256];
    const ssize_t count = read(descriptor, buffer, sizeof(buffer));
    if (count > 0)
    {
      for (ssize_t index = 0; index < count; ++index)
      {
        if (buffer[index] == 0)
        {
          bool crc_error = false;
          if (zyron_protocol::parsePacket(
              encoded.data(), encoded.size(), decoded.data(), packet, crc_error)) return true;
          encoded.clear();
        }
        else encoded.push_back(buffer[index]);
      }
    }
    std::this_thread::sleep_for(1ms);
  }
  return false;
}

struct PseudoTerminal
{
  int master = -1;
  std::string slave_name;
  PseudoTerminal()
  {
    int slave = -1;
    char name[128]{};
    EXPECT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
    slave_name = name;
    close(slave);
    EXPECT_NE(fcntl(master, F_SETFL, O_NONBLOCK), -1);
  }
  ~PseudoTerminal() {if (master >= 0) close(master);}
};

std::vector<uint8_t> hello_frame(uint16_t sequence)
{
  zyron_protocol::HelloPayload hello;
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t size = 0;
  EXPECT_TRUE(zyron_protocol::encodeHello(hello, payload, sizeof(payload), size));
  return make_frame(zyron_protocol::MessageType::HELLO, sequence, payload, size);
}

std::vector<uint8_t> telemetry_frame(
  uint16_t sequence, uint16_t flags =
    zyron_protocol::STATUS_CONFIGURED | zyron_protocol::STATUS_IMU_READY)
{
  zyron_protocol::TelemetryPayload telemetry;
  telemetry.left_encoder_count = 228;
  telemetry.right_encoder_count = 448;
  telemetry.left_velocity_rad_s = 2.0F;
  telemetry.right_velocity_rad_s = 4.0F;
  telemetry.qw = 1.0F;
  telemetry.ax = 1.0F;
  telemetry.ay = 2.0F;
  telemetry.az = 3.0F;
  telemetry.gx = 4.0F;
  telemetry.gy = 5.0F;
  telemetry.gz = 6.0F;
  telemetry.left_applied_pwm = 70;
  telemetry.right_applied_pwm = 80;
  telemetry.status_flags = flags;
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t size = 0;
  EXPECT_TRUE(zyron_protocol::encodeTelemetry(telemetry, payload, sizeof(payload), size));
  return make_frame(zyron_protocol::MessageType::TELEMETRY, sequence, payload, size);
}

uint16_t complete_handshake(int master, ZyronSerialDriver & driver, uint16_t hello_sequence = 0)
{
  write_all(master, hello_frame(hello_sequence));
  zyron_protocol::PacketView packet;
  std::array<uint8_t, zyron_protocol::kMaxDecodedFrame> decoded{};
  EXPECT_TRUE(read_packet(master, packet, decoded, 500ms));
  EXPECT_EQ(packet.type, zyron_protocol::MessageType::CONFIG);
  zyron_protocol::ConfigPayload config;
  EXPECT_TRUE(zyron_protocol::decodeConfig(packet.payload, packet.payload_size, config));
  EXPECT_FLOAT_EQ(config.left_counts_per_rev, 228.0F);

  zyron_protocol::ConfigAckPayload ack;
  ack.config_crc = zyron_protocol::crc16Ccitt(packet.payload, packet.payload_size);
  ack.result = static_cast<uint8_t>(zyron_protocol::ConfigResult::ACCEPTED);
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t payload_size = 0;
  EXPECT_TRUE(zyron_protocol::encodeConfigAck(ack, payload, sizeof(payload), payload_size));
  write_all(master, make_frame(
    zyron_protocol::MessageType::CONFIG_ACK, hello_sequence + 1, payload, payload_size));
  const auto deadline = std::chrono::steady_clock::now() + 250ms;
  while (!driver.configured() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(1ms);
  EXPECT_TRUE(driver.configured());
  return static_cast<uint16_t>(hello_sequence + 2);
}

TEST(BinaryProtocolTest, CobsCrcAndPayloadRoundTrip)
{
  zyron_protocol::CommandPayload command;
  command.left_target_rad_s = -3.25F;
  command.right_target_rad_s = 5.5F;
  command.enable = 1;
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t payload_size = 0;
  ASSERT_TRUE(zyron_protocol::encodeCommand(command, payload, sizeof(payload), payload_size));
  auto frame = make_frame(zyron_protocol::MessageType::COMMAND, 65535, payload, payload_size);
  ASSERT_FALSE(frame.empty());
  ASSERT_EQ(frame.back(), 0);

  zyron_protocol::PacketView packet;
  std::array<uint8_t, zyron_protocol::kMaxDecodedFrame> decoded{};
  bool crc_error = false;
  ASSERT_TRUE(zyron_protocol::parsePacket(
    frame.data(), frame.size() - 1, decoded.data(), packet, crc_error));
  EXPECT_EQ(packet.sequence, 65535);
  zyron_protocol::CommandPayload parsed;
  ASSERT_TRUE(zyron_protocol::decodeCommand(packet.payload, packet.payload_size, parsed));
  EXPECT_FLOAT_EQ(parsed.left_target_rad_s, -3.25F);
  EXPECT_FLOAT_EQ(parsed.right_target_rad_s, 5.5F);
  EXPECT_EQ(parsed.enable, 1);

  frame[5] ^= 0x40;
  EXPECT_FALSE(zyron_protocol::parsePacket(
    frame.data(), frame.size() - 1, decoded.data(), packet, crc_error));
}

TEST(BinaryProtocolTest, RejectsMalformedAndOverlongFrames)
{
  std::array<uint8_t, zyron_protocol::kMaxDecodedFrame> decoded{};
  zyron_protocol::PacketView packet;
  bool crc_error = false;
  const uint8_t malformed[] = {2, 1, 2};
  EXPECT_FALSE(zyron_protocol::parsePacket(
    malformed, sizeof(malformed), decoded.data(), packet, crc_error));
  std::array<uint8_t, zyron_protocol::kMaxEncodedFrame + 1> overlong{};
  EXPECT_FALSE(zyron_protocol::parsePacket(
    overlong.data(), overlong.size(), decoded.data(), packet, crc_error));

  zyron_protocol::HelloPayload hello;
  uint8_t payload[zyron_protocol::kMaxPayload];
  std::size_t payload_size = 0;
  ASSERT_TRUE(zyron_protocol::encodeHello(hello, payload, sizeof(payload), payload_size));
  auto frame = make_frame(zyron_protocol::MessageType::HELLO, 1, payload, payload_size);
  std::array<uint8_t, zyron_protocol::kMaxDecodedFrame> raw{};
  const std::size_t raw_size = zyron_protocol::cobsDecode(
    frame.data(), frame.size() - 1, raw.data(), raw.size());
  ASSERT_GT(raw_size, zyron_protocol::kHeaderSize);
  raw[2] = zyron_protocol::kVersion + 1;
  std::array<uint8_t, zyron_protocol::kMaxEncodedFrame> wrong_version{};
  const std::size_t wrong_size = zyron_protocol::cobsEncode(
    raw.data(), raw_size, wrong_version.data(), wrong_version.size());
  EXPECT_FALSE(zyron_protocol::parsePacket(
    wrong_version.data(), wrong_size, decoded.data(), packet, crc_error));
}

TEST(ZyronSerialDriverTest, ValidatesConfiguration)
{
  DriverConfig config;
  EXPECT_TRUE(ZyronSerialDriver::validate_config(config));
  config.port.clear();
  EXPECT_FALSE(ZyronSerialDriver::validate_config(config));
  config.port = "/dev/null";
  config.baud_rate = 12345;
  EXPECT_FALSE(ZyronSerialDriver::validate_config(config));
  config.baud_rate = 460800;
  config.start_threshold_rad_s = config.stop_threshold_rad_s;
  EXPECT_FALSE(ZyronSerialDriver::validate_config(config));
  config.start_threshold_rad_s = 0.5F;
  config.left_pwm_min = 256;
  EXPECT_FALSE(ZyronSerialDriver::validate_config(config));
}

TEST(ZyronSerialDriverTest, ReportsSerialOpenFailure)
{
  DriverConfig config;
  config.port = "/dev/zyron-port-that-does-not-exist";
  ZyronSerialDriver driver(config);
  EXPECT_FALSE(driver.start());
  EXPECT_FALSE(driver.startup_error().empty());
}

TEST(ZyronSerialDriverTest, DetectsFreshness)
{
  DriverSample sample;
  const auto stamp = std::chrono::steady_clock::now();
  sample.wheel_valid = true;
  sample.imu_valid = true;
  sample.wheel_stamp = stamp;
  sample.imu_stamp = stamp;
  EXPECT_TRUE(ZyronSerialDriver::wheel_state_is_fresh(sample, 250ms, stamp + 250ms));
  EXPECT_FALSE(ZyronSerialDriver::wheel_state_is_fresh(sample, 250ms, stamp + 251ms));
  EXPECT_TRUE(ZyronSerialDriver::imu_state_is_fresh(sample, 250ms, stamp + 250ms));
}

TEST(ZyronSerialDriverTest, HandshakesAndExchangesWheelSpeedData)
{
  PseudoTerminal terminal;
  DriverConfig config;
  config.port = terminal.slave_name;
  ZyronSerialDriver driver(config);
  ASSERT_TRUE(driver.start());
  uint16_t sequence = complete_handshake(terminal.master, driver);
  for (int index = 0; index < 3; ++index) write_all(terminal.master, telemetry_frame(sequence++));

  DriverSample sample;
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline)
  {
    driver.latest_sample(sample);
    if (sample.motion_allowed) break;
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_TRUE(sample.config_acknowledged);
  EXPECT_TRUE(sample.motion_allowed);
  EXPECT_NEAR(sample.wheel_position[0], 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(sample.wheel_position[1], 4.0 * M_PI, 1e-6);
  EXPECT_DOUBLE_EQ(sample.wheel_velocity[1], 4.0);
  EXPECT_DOUBLE_EQ(sample.imu_angular_velocity[2], 6.0);

  driver.set_velocity_commands(3.0, -4.0);
  zyron_protocol::PacketView packet;
  std::array<uint8_t, zyron_protocol::kMaxDecodedFrame> decoded{};
  bool found_enabled = false;
  const auto command_deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < command_deadline)
  {
    if (!read_packet(terminal.master, packet, decoded, 30ms) ||
      packet.type != zyron_protocol::MessageType::COMMAND) continue;
    zyron_protocol::CommandPayload command;
    ASSERT_TRUE(zyron_protocol::decodeCommand(packet.payload, packet.payload_size, command));
    if (command.enable)
    {
      EXPECT_FLOAT_EQ(command.left_target_rad_s, 3.0F);
      EXPECT_FLOAT_EQ(command.right_target_rad_s, -4.0F);
      found_enabled = true;
      break;
    }
  }
  EXPECT_TRUE(found_enabled);
  driver.stop();
}

TEST(ZyronSerialDriverTest, SuppressesStaleTelemetryAndRearmsAfterThreePackets)
{
  PseudoTerminal terminal;
  DriverConfig config;
  config.port = terminal.slave_name;
  config.state_timeout = 50ms;
  config.imu_timeout = 50ms;
  ZyronSerialDriver driver(config);
  ASSERT_TRUE(driver.start());
  uint16_t sequence = complete_handshake(terminal.master, driver);
  for (int index = 0; index < 3; ++index) write_all(terminal.master, telemetry_frame(sequence++));
  driver.set_velocity_commands(2.0, 2.0);
  std::this_thread::sleep_for(80ms);
  EXPECT_EQ(driver.motion_inhibit_reason(), MotionInhibitReason::TELEMETRY_STALE);

  write_all(terminal.master, telemetry_frame(sequence++));
  write_all(terminal.master, telemetry_frame(sequence++));
  std::this_thread::sleep_for(20ms);
  EXPECT_NE(driver.motion_inhibit_reason(), MotionInhibitReason::NONE);
  write_all(terminal.master, telemetry_frame(sequence++));
  const auto deadline = std::chrono::steady_clock::now() + 100ms;
  while (driver.motion_inhibit_reason() != MotionInhibitReason::NONE &&
    std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(2ms);
  EXPECT_EQ(driver.motion_inhibit_reason(), MotionInhibitReason::NONE);
  driver.stop();
}

TEST(ZyronSerialDriverTest, HandlesFragmentationCorruptionAndBurstTraffic)
{
  PseudoTerminal terminal;
  DriverConfig config;
  config.port = terminal.slave_name;
  ZyronSerialDriver driver(config);
  ASSERT_TRUE(driver.start());

  const auto hello = hello_frame(0);
  for (std::size_t offset = 0; offset < hello.size(); offset += 3)
  {
    const std::size_t count = std::min<std::size_t>(3, hello.size() - offset);
    ASSERT_EQ(write(terminal.master, hello.data() + offset, count), static_cast<ssize_t>(count));
    std::this_thread::sleep_for(4ms);
  }
  zyron_protocol::PacketView config_packet;
  std::array<uint8_t, zyron_protocol::kMaxDecodedFrame> decoded{};
  ASSERT_TRUE(read_packet(terminal.master, config_packet, decoded, 500ms));
  zyron_protocol::ConfigAckPayload ack;
  ack.config_crc = zyron_protocol::crc16Ccitt(config_packet.payload, config_packet.payload_size);
  ack.result = static_cast<uint8_t>(zyron_protocol::ConfigResult::ACCEPTED);
  uint8_t ack_payload[zyron_protocol::kMaxPayload];
  std::size_t ack_size = 0;
  ASSERT_TRUE(zyron_protocol::encodeConfigAck(ack, ack_payload, sizeof(ack_payload), ack_size));
  write_all(terminal.master, make_frame(zyron_protocol::MessageType::CONFIG_ACK, 1, ack_payload, ack_size));
  const auto configured_deadline = std::chrono::steady_clock::now() + 250ms;
  while (!driver.configured() && std::chrono::steady_clock::now() < configured_deadline)
    std::this_thread::sleep_for(1ms);
  ASSERT_TRUE(driver.configured());

  auto corrupt = telemetry_frame(2);
  corrupt[4] ^= 0x55;
  write_all(terminal.master, corrupt);
  std::vector<uint8_t> overlong(150, 0x11);
  overlong.push_back(0);
  write_all(terminal.master, overlong);
  for (uint16_t sequence = 3; sequence < 23; ++sequence)
    write_all(terminal.master, telemetry_frame(sequence));

  DriverSample sample;
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline)
  {
    driver.latest_sample(sample);
    if (sample.wheel_sequence >= 10) break;
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_GE(sample.wheel_sequence, 10U);
  EXPECT_GE(sample.host_crc_errors + sample.host_frame_errors, 2U);
  EXPECT_TRUE(driver.healthy());
  driver.stop();
}

}  // namespace
}  // namespace zyron_control
