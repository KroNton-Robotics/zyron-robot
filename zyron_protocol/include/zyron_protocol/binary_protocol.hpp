#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace zyron_protocol
{

constexpr uint16_t kMagic = 0x595A;
constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderSize = 8;
constexpr size_t kCrcSize = 2;
constexpr size_t kMaxDecodedFrame = 96;
constexpr size_t kMaxPayload = kMaxDecodedFrame - kHeaderSize - kCrcSize;
constexpr size_t kMaxEncodedFrame = kMaxDecodedFrame + 2;

enum class MessageType : uint8_t
{
  HELLO = 0x01,
  CONFIG = 0x02,
  CONFIG_ACK = 0x03,
  COMMAND = 0x10,
  TELEMETRY = 0x20,
};

enum StatusFlag : uint16_t
{
  STATUS_CONFIGURED = 1U << 0,
  STATUS_IMU_READY = 1U << 1,
  STATUS_IMU_STALE = 1U << 2,
  STATUS_COMMAND_STALE = 1U << 3,
  STATUS_LEFT_SATURATED = 1U << 4,
  STATUS_RIGHT_SATURATED = 1U << 5,
};

enum class ConfigResult : uint8_t
{
  ACCEPTED = 0,
  INVALID = 1,
  UNSUPPORTED_VERSION = 2,
};

struct HelloPayload
{
  uint8_t firmware_major{1};
  uint8_t firmware_minor{0};
  uint8_t firmware_patch{0};
  uint8_t capabilities{1};
  uint32_t uptime_ms{0};
};

struct ConfigPayload
{
  float left_counts_per_rev{228.0F};
  float right_counts_per_rev{224.0F};
  float max_speed_rad_s{20.94F};
  float start_threshold_rad_s{0.5F};
  float stop_threshold_rad_s{0.2F};
  uint8_t left_pwm_min{60};
  uint8_t right_pwm_min{60};
  uint8_t pwm_max{255};
  uint8_t reserved{0};
  float left_kp{1.0F};
  float left_ki{2.0F};
  float right_kp{1.0F};
  float right_ki{2.0F};
  float max_acceleration_rad_s2{15.0F};
  float velocity_filter_tau_s{0.1F};
  uint16_t command_watchdog_ms{200};
  uint16_t reserved2{0};
};

struct ConfigAckPayload
{
  uint16_t config_crc{0};
  uint8_t result{static_cast<uint8_t>(ConfigResult::INVALID)};
  uint8_t reserved{0};
};

struct CommandPayload
{
  float left_target_rad_s{0.0F};
  float right_target_rad_s{0.0F};
  uint8_t enable{0};
  uint8_t reserved[3]{0, 0, 0};
};

struct TelemetryPayload
{
  uint32_t uptime_ms{0};
  int32_t left_encoder_count{0};
  int32_t right_encoder_count{0};
  float left_velocity_rad_s{0.0F};
  float right_velocity_rad_s{0.0F};
  float qx{0.0F};
  float qy{0.0F};
  float qz{0.0F};
  float qw{1.0F};
  float ax{0.0F};
  float ay{0.0F};
  float az{0.0F};
  float gx{0.0F};
  float gy{0.0F};
  float gz{0.0F};
  int16_t left_applied_pwm{0};
  int16_t right_applied_pwm{0};
  uint16_t status_flags{0};
  uint16_t last_command_sequence{0};
  uint16_t rx_crc_errors{0};
  uint16_t rx_frame_errors{0};
};

struct PacketView
{
  MessageType type{MessageType::HELLO};
  uint16_t sequence{0};
  const uint8_t * payload{nullptr};
  uint16_t payload_size{0};
};

inline uint16_t crc16Ccitt(const uint8_t * data, size_t size)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline void putU16(uint8_t * output, uint16_t value)
{
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8);
}

inline void putU32(uint8_t * output, uint32_t value)
{
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8);
  output[2] = static_cast<uint8_t>(value >> 16);
  output[3] = static_cast<uint8_t>(value >> 24);
}

inline uint16_t getU16(const uint8_t * input)
{
  return static_cast<uint16_t>(input[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(input[1]) << 8);
}

inline uint32_t getU32(const uint8_t * input)
{
  return static_cast<uint32_t>(input[0]) |
         (static_cast<uint32_t>(input[1]) << 8) |
         (static_cast<uint32_t>(input[2]) << 16) |
         (static_cast<uint32_t>(input[3]) << 24);
}

class Writer
{
public:
  Writer(uint8_t * data, size_t capacity) : data_(data), capacity_(capacity) {}

  bool u8(uint8_t value) {return bytes(&value, 1);}
  bool u16(uint16_t value) {uint8_t raw[2]; putU16(raw, value); return bytes(raw, 2);}
  bool u32(uint32_t value) {uint8_t raw[4]; putU32(raw, value); return bytes(raw, 4);}
  bool i16(int16_t value) {return u16(static_cast<uint16_t>(value));}
  bool i32(int32_t value) {return u32(static_cast<uint32_t>(value));}
  bool f32(float value)
  {
    uint32_t raw = 0;
    static_assert(sizeof(float) == sizeof(uint32_t), "protocol requires 32-bit float");
    memcpy(&raw, &value, sizeof(raw));
    return u32(raw);
  }
  bool bytes(const uint8_t * source, size_t count)
  {
    if (position_ + count > capacity_) {return false;}
    memcpy(data_ + position_, source, count);
    position_ += count;
    return true;
  }
  size_t size() const {return position_;}

private:
  uint8_t * data_;
  size_t capacity_;
  size_t position_{0};
};

class Reader
{
public:
  Reader(const uint8_t * data, size_t size) : data_(data), size_(size) {}
  bool u8(uint8_t & value) {return bytes(&value, 1);}
  bool u16(uint16_t & value) {uint8_t raw[2]; if (!bytes(raw, 2)) return false; value = getU16(raw); return true;}
  bool u32(uint32_t & value) {uint8_t raw[4]; if (!bytes(raw, 4)) return false; value = getU32(raw); return true;}
  bool i16(int16_t & value) {uint16_t raw; if (!u16(raw)) return false; value = static_cast<int16_t>(raw); return true;}
  bool i32(int32_t & value) {uint32_t raw; if (!u32(raw)) return false; value = static_cast<int32_t>(raw); return true;}
  bool f32(float & value)
  {
    uint32_t raw;
    if (!u32(raw)) return false;
    memcpy(&value, &raw, sizeof(value));
    return true;
  }
  bool bytes(uint8_t * destination, size_t count)
  {
    if (position_ + count > size_) return false;
    memcpy(destination, data_ + position_, count);
    position_ += count;
    return true;
  }
  bool complete() const {return position_ == size_;}

private:
  const uint8_t * data_;
  size_t size_;
  size_t position_{0};
};

inline size_t cobsEncode(const uint8_t * input, size_t size, uint8_t * output, size_t capacity)
{
  if (capacity == 0) return 0;
  size_t read_index = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  uint8_t code = 1;
  while (read_index < size)
  {
    if (input[read_index] == 0)
    {
      if (code_index >= capacity) return 0;
      output[code_index] = code;
      code = 1;
      code_index = write_index++;
      if (write_index > capacity) return 0;
      ++read_index;
    }
    else
    {
      if (write_index >= capacity) return 0;
      output[write_index++] = input[read_index++];
      if (++code == 0xFF)
      {
        output[code_index] = code;
        code = 1;
        code_index = write_index++;
        if (write_index > capacity) return 0;
      }
    }
  }
  if (code_index >= capacity) return 0;
  output[code_index] = code;
  return write_index;
}

inline size_t cobsDecode(const uint8_t * input, size_t size, uint8_t * output, size_t capacity)
{
  size_t read_index = 0;
  size_t write_index = 0;
  while (read_index < size)
  {
    const uint8_t code = input[read_index++];
    if (code == 0) return 0;
    const size_t count = static_cast<size_t>(code - 1);
    if (read_index + count > size || write_index + count > capacity) return 0;
    for (size_t i = 0; i < count; ++i) output[write_index++] = input[read_index++];
    if (code != 0xFF && read_index < size)
    {
      if (write_index >= capacity) return 0;
      output[write_index++] = 0;
    }
  }
  return write_index;
}

inline size_t makePacket(
  MessageType type, uint16_t sequence, const uint8_t * payload, uint16_t payload_size,
  uint8_t * output, size_t capacity)
{
  const size_t raw_size = kHeaderSize + payload_size + kCrcSize;
  if (payload_size > kMaxPayload || raw_size > kMaxDecodedFrame || capacity < raw_size + 2) return 0;
  uint8_t raw[kMaxDecodedFrame];
  putU16(raw, kMagic);
  raw[2] = kVersion;
  raw[3] = static_cast<uint8_t>(type);
  putU16(raw + 4, sequence);
  putU16(raw + 6, payload_size);
  if (payload_size != 0) memcpy(raw + kHeaderSize, payload, payload_size);
  putU16(raw + kHeaderSize + payload_size, crc16Ccitt(raw, kHeaderSize + payload_size));
  const size_t encoded_size = cobsEncode(raw, raw_size, output, capacity - 1);
  if (encoded_size == 0 || encoded_size >= capacity) return 0;
  output[encoded_size] = 0;
  return encoded_size + 1;
}

inline bool parsePacket(
  const uint8_t * encoded, size_t encoded_size, uint8_t * decoded,
  PacketView & packet, bool & crc_error)
{
  crc_error = false;
  if (encoded_size == 0 || encoded_size > kMaxEncodedFrame) return false;
  const size_t decoded_size = cobsDecode(encoded, encoded_size, decoded, kMaxDecodedFrame);
  if (decoded_size < kHeaderSize + kCrcSize) return false;
  const uint16_t payload_size = getU16(decoded + 6);
  if (getU16(decoded) != kMagic || decoded[2] != kVersion ||
    payload_size > kMaxPayload || decoded_size != kHeaderSize + payload_size + kCrcSize)
  {
    return false;
  }
  const uint16_t expected_crc = getU16(decoded + kHeaderSize + payload_size);
  if (crc16Ccitt(decoded, kHeaderSize + payload_size) != expected_crc)
  {
    crc_error = true;
    return false;
  }
  packet.type = static_cast<MessageType>(decoded[3]);
  packet.sequence = getU16(decoded + 4);
  packet.payload = decoded + kHeaderSize;
  packet.payload_size = payload_size;
  return true;
}

inline bool encodeHello(const HelloPayload & value, uint8_t * output, size_t capacity, size_t & size)
{
  Writer writer(output, capacity);
  const bool ok = writer.u8(value.firmware_major) && writer.u8(value.firmware_minor) &&
    writer.u8(value.firmware_patch) && writer.u8(value.capabilities) && writer.u32(value.uptime_ms);
  size = writer.size(); return ok;
}

inline bool decodeHello(const uint8_t * data, size_t size, HelloPayload & value)
{
  Reader reader(data, size);
  return reader.u8(value.firmware_major) && reader.u8(value.firmware_minor) &&
    reader.u8(value.firmware_patch) && reader.u8(value.capabilities) && reader.u32(value.uptime_ms) && reader.complete();
}

inline bool encodeConfig(const ConfigPayload & v, uint8_t * output, size_t capacity, size_t & size)
{
  Writer w(output, capacity);
  const bool ok = w.f32(v.left_counts_per_rev) && w.f32(v.right_counts_per_rev) &&
    w.f32(v.max_speed_rad_s) && w.f32(v.start_threshold_rad_s) && w.f32(v.stop_threshold_rad_s) &&
    w.u8(v.left_pwm_min) && w.u8(v.right_pwm_min) && w.u8(v.pwm_max) && w.u8(v.reserved) &&
    w.f32(v.left_kp) && w.f32(v.left_ki) && w.f32(v.right_kp) && w.f32(v.right_ki) &&
    w.f32(v.max_acceleration_rad_s2) && w.f32(v.velocity_filter_tau_s) &&
    w.u16(v.command_watchdog_ms) && w.u16(v.reserved2);
  size = w.size(); return ok;
}

inline bool decodeConfig(const uint8_t * data, size_t size, ConfigPayload & v)
{
  Reader r(data, size);
  return r.f32(v.left_counts_per_rev) && r.f32(v.right_counts_per_rev) &&
    r.f32(v.max_speed_rad_s) && r.f32(v.start_threshold_rad_s) && r.f32(v.stop_threshold_rad_s) &&
    r.u8(v.left_pwm_min) && r.u8(v.right_pwm_min) && r.u8(v.pwm_max) && r.u8(v.reserved) &&
    r.f32(v.left_kp) && r.f32(v.left_ki) && r.f32(v.right_kp) && r.f32(v.right_ki) &&
    r.f32(v.max_acceleration_rad_s2) && r.f32(v.velocity_filter_tau_s) &&
    r.u16(v.command_watchdog_ms) && r.u16(v.reserved2) && r.complete();
}

inline bool encodeConfigAck(const ConfigAckPayload & v, uint8_t * output, size_t capacity, size_t & size)
{Writer w(output, capacity); const bool ok = w.u16(v.config_crc) && w.u8(v.result) && w.u8(v.reserved); size = w.size(); return ok;}
inline bool decodeConfigAck(const uint8_t * data, size_t size, ConfigAckPayload & v)
{Reader r(data, size); return r.u16(v.config_crc) && r.u8(v.result) && r.u8(v.reserved) && r.complete();}

inline bool encodeCommand(const CommandPayload & v, uint8_t * output, size_t capacity, size_t & size)
{Writer w(output, capacity); const bool ok = w.f32(v.left_target_rad_s) && w.f32(v.right_target_rad_s) && w.u8(v.enable) && w.bytes(v.reserved, 3); size = w.size(); return ok;}
inline bool decodeCommand(const uint8_t * data, size_t size, CommandPayload & v)
{Reader r(data, size); return r.f32(v.left_target_rad_s) && r.f32(v.right_target_rad_s) && r.u8(v.enable) && r.bytes(v.reserved, 3) && r.complete();}

inline bool encodeTelemetry(const TelemetryPayload & v, uint8_t * output, size_t capacity, size_t & size)
{
  Writer w(output, capacity);
  const bool ok = w.u32(v.uptime_ms) && w.i32(v.left_encoder_count) && w.i32(v.right_encoder_count) &&
    w.f32(v.left_velocity_rad_s) && w.f32(v.right_velocity_rad_s) &&
    w.f32(v.qx) && w.f32(v.qy) && w.f32(v.qz) && w.f32(v.qw) &&
    w.f32(v.ax) && w.f32(v.ay) && w.f32(v.az) && w.f32(v.gx) && w.f32(v.gy) && w.f32(v.gz) &&
    w.i16(v.left_applied_pwm) && w.i16(v.right_applied_pwm) && w.u16(v.status_flags) &&
    w.u16(v.last_command_sequence) && w.u16(v.rx_crc_errors) && w.u16(v.rx_frame_errors);
  size = w.size(); return ok;
}

inline bool decodeTelemetry(const uint8_t * data, size_t size, TelemetryPayload & v)
{
  Reader r(data, size);
  return r.u32(v.uptime_ms) && r.i32(v.left_encoder_count) && r.i32(v.right_encoder_count) &&
    r.f32(v.left_velocity_rad_s) && r.f32(v.right_velocity_rad_s) &&
    r.f32(v.qx) && r.f32(v.qy) && r.f32(v.qz) && r.f32(v.qw) &&
    r.f32(v.ax) && r.f32(v.ay) && r.f32(v.az) && r.f32(v.gx) && r.f32(v.gy) && r.f32(v.gz) &&
    r.i16(v.left_applied_pwm) && r.i16(v.right_applied_pwm) && r.u16(v.status_flags) &&
    r.u16(v.last_command_sequence) && r.u16(v.rx_crc_errors) && r.u16(v.rx_frame_errors) && r.complete();
}

}  // namespace zyron_protocol
