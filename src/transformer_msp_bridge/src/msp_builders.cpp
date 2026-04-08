// 建立基础msp通信的字符串包
// Implements helpers for constructing MSP frames used for outbound traffic.
#include "transformer_msp_bridge/msp_builders.hpp"
#include "transformer_msp_bridge/crc.hpp"
#include "transformer_msp_bridge/msp_registry.hpp"

namespace transformer_msp_bridge
{

  std::vector<uint8_t> buildPacket(uint8_t command, const std::vector<uint16_t> &rc_channel_values)
  {
    // Serialize 16-bit RC channel values little-endian into a payload buffer.
    std::vector<uint8_t> payload_bytes;
    payload_bytes.reserve(rc_channel_values.size() * 2);
    for (uint16_t channel_value : rc_channel_values)
    {
      payload_bytes.push_back(static_cast<uint8_t>(channel_value & 0xFF));
      payload_bytes.push_back(static_cast<uint8_t>((channel_value >> 8) & 0xFF));
    }
    return buildPacketRaw(command, payload_bytes);
  }

  std::vector<uint8_t> buildPacketRaw(uint8_t command, const std::vector<uint8_t> &payload)
  {
    const uint8_t payload_size = static_cast<uint8_t>(payload.size());
    uint8_t checksum = payload_size ^ command;
    for (uint8_t byte_value : payload)
      checksum ^= byte_value;

    std::vector<uint8_t> frame;
    frame.reserve(6 + payload.size()); // header + payload + checksum

    // Preamble '$','M','<'
    frame.push_back('$');
    frame.push_back('M');
    frame.push_back('<');
    frame.push_back(payload_size);
    frame.push_back(command);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(checksum);
    return frame;
  }

  std::vector<uint8_t> buildPacketV2(uint16_t command, const std::vector<uint8_t> &payload, uint8_t flags)
  {
    using namespace crc8_dvb_s2;

    const uint16_t payload_length = static_cast<uint16_t>(payload.size());
    uint8_t crc8 = 0;
    auto accumulate_crc = [&](uint8_t byte_value)
    {
      crc8 = update(crc8, byte_value);
    };

    std::vector<uint8_t> frame_buffer;
    frame_buffer.reserve(8 + payload.size()); // header + payload + crc

    // Preamble '$','X','<'
    frame_buffer.push_back('$');
    frame_buffer.push_back('X');
    frame_buffer.push_back('<');

    // Flags
    frame_buffer.push_back(flags);
    accumulate_crc(flags);

    // Command (little endian)
    const uint8_t command_low_byte = static_cast<uint8_t>(command & 0xFF);
    const uint8_t command_high_byte = static_cast<uint8_t>((command >> 8) & 0xFF);
    frame_buffer.push_back(command_low_byte);
    accumulate_crc(command_low_byte);
    frame_buffer.push_back(command_high_byte);
    accumulate_crc(command_high_byte);

    // Payload length (little endian)
    const uint8_t length_low_byte = static_cast<uint8_t>(payload_length & 0xFF);
    const uint8_t length_high_byte = static_cast<uint8_t>((payload_length >> 8) & 0xFF);
    frame_buffer.push_back(length_low_byte);
    accumulate_crc(length_low_byte);
    frame_buffer.push_back(length_high_byte);
    accumulate_crc(length_high_byte);

    // Payload bytes
    for (uint8_t byte_value : payload)
    {
      frame_buffer.push_back(byte_value);
      accumulate_crc(byte_value);
    }

    // Final CRC
    frame_buffer.push_back(crc8);
    return frame_buffer;
  }

  std::vector<uint8_t> buildPacketV2OverV1(uint16_t command, const std::vector<uint8_t> &payload, uint8_t flags)
  {
    using namespace crc8_dvb_s2;

    const uint16_t payload_length = static_cast<uint16_t>(payload.size());

    // Build the MSP v2 header fields that will be tunneled inside a v1 frame.
    uint8_t v2_header[5];
    v2_header[0] = flags;
    v2_header[1] = static_cast<uint8_t>(command & 0xFF);
    v2_header[2] = static_cast<uint8_t>((command >> 8) & 0xFF);
    v2_header[3] = static_cast<uint8_t>(payload_length & 0xFF);
    v2_header[4] = static_cast<uint8_t>((payload_length >> 8) & 0xFF);

    // Compute CRC over header then payload per MSP v2 spec.
    uint8_t crc8 = 0;
    for (uint8_t byte_value : v2_header)
    {
      crc8 = update(crc8, byte_value);
    }
    for (uint8_t byte_value : payload)
    {
      crc8 = update(crc8, byte_value);
    }

    // Assemble inner (v2) frame: header + payload + crc
    std::vector<uint8_t> inner_frame;
    inner_frame.reserve(5 + payload.size() + 1);
    inner_frame.insert(inner_frame.end(), v2_header, v2_header + 5);
    inner_frame.insert(inner_frame.end(), payload.begin(), payload.end());
    inner_frame.push_back(crc8);

    // Wrap the v2 frame inside a v1 frame with command MSP_V2_FRAME_ID
  return buildPacketRaw(static_cast<uint8_t>(msp::kMspV2FrameId), inner_frame);
  }

} // namespace transformer_msp_bridge
