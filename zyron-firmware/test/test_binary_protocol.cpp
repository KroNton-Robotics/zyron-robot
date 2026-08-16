#include <cassert>
#include <cstring>

#include <zyron_protocol/binary_protocol.hpp>

int main()
{
  zyron_protocol::ConfigPayload config;
  uint8_t payload[zyron_protocol::kMaxPayload];
  size_t payload_size = 0;
  assert(zyron_protocol::encodeConfig(config, payload, sizeof(payload), payload_size));
  assert(payload_size == 52);

  uint8_t encoded[zyron_protocol::kMaxEncodedFrame];
  const size_t frame_size = zyron_protocol::makePacket(
    zyron_protocol::MessageType::CONFIG, 65535, payload,
    static_cast<uint16_t>(payload_size), encoded, sizeof(encoded));
  assert(frame_size > 0);
  assert(encoded[frame_size - 1] == 0);

  uint8_t decoded[zyron_protocol::kMaxDecodedFrame];
  zyron_protocol::PacketView packet;
  bool crc_error = false;
  assert(zyron_protocol::parsePacket(
    encoded, frame_size - 1, decoded, packet, crc_error));
  assert(packet.sequence == 65535);
  assert(packet.type == zyron_protocol::MessageType::CONFIG);
  zyron_protocol::ConfigPayload parsed;
  assert(zyron_protocol::decodeConfig(packet.payload, packet.payload_size, parsed));
  assert(parsed.left_counts_per_rev == config.left_counts_per_rev);

  encoded[4] ^= 0x20;
  assert(!zyron_protocol::parsePacket(
    encoded, frame_size - 1, decoded, packet, crc_error));

  uint8_t raw[zyron_protocol::kMaxDecodedFrame]{};
  for (size_t index = 0; index < sizeof(raw); ++index)
    raw[index] = static_cast<uint8_t>(index % 7 == 0 ? 0 : index);
  uint8_t cobs[zyron_protocol::kMaxEncodedFrame];
  const size_t cobs_size = zyron_protocol::cobsEncode(raw, sizeof(raw), cobs, sizeof(cobs));
  assert(cobs_size > 0);
  uint8_t round_trip[zyron_protocol::kMaxDecodedFrame];
  assert(zyron_protocol::cobsDecode(cobs, cobs_size, round_trip, sizeof(round_trip)) == sizeof(raw));
  assert(std::memcmp(raw, round_trip, sizeof(raw)) == 0);
  return 0;
}
