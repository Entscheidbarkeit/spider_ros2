// 按字节流parse通讯帧
// Implements the streaming MultiWii/iNav MSP parser used by the bridge.
// -------------------------
// This file implements a streaming parser for the MultiWii/iNav MSP (Multiwii Serial Protocol)
// supporting:
//   * MSP v1 frames
//   * MSP v2 native frames
//   * MSP v2 frames tunneled inside a v1 frame (command 255)
//
// Design notes:
//  * The parser is byte-driven: each incoming byte advances a 1-based frame_pos_.
//  * Once a frame start ('$') is detected we classify version using byte 2 ('M' or 'X').
//  * For each version we maintain minimal positional logic instead of a full enum state machine.
//  * Expected total frame length is derived as soon as the payload length is known; from then on
//    we can identify the final checksum/CRC byte position directly.
//  * For v1 we compute an XOR checksum over length, command and payload. If it matches at frame end
//    we either emit a native v1 packet or attempt to decode a tunneled v2.
//  * For v2 we compute a CRC8 (DVB-S2 polynomial) over flags, command (LE), length (LE) and payload.
//  * A unified finishFrame() helper centralizes terminal emission, statistics updates and reset.
//  * Hardening: maximum payload size guard, malformed header tracking, oversized frame drop counter.
//  * Parser keeps a frame_trace_ mainly for tests / diagnostics (can be trimmed for production if desired).
//  * Zero-copy style access to the accumulating payload is exposed via payloadData()/payloadSize().
//
// Error handling philosophy:
//  * On any structural inconsistency we reset() immediately (fast-fail recovery to search for next '$').
//  * Silent drops for v1 checksum mismatch (mirrors typical MSP behavior). V2 CRC failure increments a counter.
//  * Tunneled v2 CRC failure increments a separate counter without emitting a partial packet.

#include "transformer_msp_bridge/msp_parser.hpp"
#include "transformer_msp_bridge/crc.hpp"
#include "transformer_msp_bridge/msp_registry.hpp"
#include "transformer_msp_bridge/msp_utils.hpp"

namespace transformer_msp_bridge
{

  MSPParser::MSPParser(PacketCallback cb) : cb_(std::move(cb)) {}

  void MSPParser::reset()
  {
    in_frame_ = false;
    current_version_ = MSPVersion::V1;
    frame_pos_ = 0;
    expected_frame_length_ = 0;
    v1_payload_length_ = 0;
    v1_command_ = 0;
    v1_checksum_ = 0;
    v2_flags_ = 0;
    v2_command_ = 0;
    v2_payload_length_ = 0;
    v2_crc_computed_ = 0;
    v2_crc_received_ = 0;
    payload_buffer_.clear();
    frame_trace_.clear();
  }

  void MSPParser::emitV1()
  {
    if (cb_)
      cb_(makePacketV1());
  }

  void MSPParser::emitV2()
  {
    if (cb_)
      cb_(makePacketV2(false));
    stats_.native_v2_rx++;
  }

  void MSPParser::finishFrame(bool ok, bool is_v2)
  {
    if (is_v2)
    {
      if (ok)
      {
        emitV2();
      }
      else
      {
        stats_.native_v2_crc_fail++;
      }
    }
    else
    { // V1 path
      if (ok)
      {
        // V1 may carry tunneled V2; handleTunneledV2() emits either tunneled V2 or native V1.
        handleTunneledV2();
      }
      // On failure we just drop silently (consistent with previous behavior that only emitted when checksum matched).
    }
    reset();
  }

  void MSPParser::handleTunneledV2()
  {
    // Layout: [flags][cmd_le(2)][len_le(2)][payload...][crc8]
  if (v1_command_ != msp::kMspV2FrameId || payload_buffer_.size() < 6)
    {
      emitV1();
      return;
    }
    const size_t flags_index = 0;
    const size_t cmd_index = 1;   // little-endian 2 bytes
    const size_t len_index = 3;   // little-endian 2 bytes
    const size_t header_size = 5; // flags + cmd(2) + len(2)
    uint16_t inner_len = 0;
    if (!readU16LE(payload_buffer_, len_index, inner_len))
    {
      emitV1();
      return;
    }
    const size_t expected_total = header_size + inner_len + 1; // +crc
    if (payload_buffer_.size() != expected_total)
    {
      emitV1();
      return;
    }
    uint8_t crc_calc = 0;
    for (size_t i = 0; i < header_size + inner_len; ++i)
      crc_calc = crc8_dvb_s2::update(crc_calc, payload_buffer_[i]);
    uint8_t crc_recv = payload_buffer_[header_size + inner_len];
    if (crc_calc != crc_recv)
    {
      stats_.tunneled_v2_crc_fail++;
      return;
    }
    MSPPacket pkt;
    pkt.version = MSPVersion::V2;
    pkt.tunneled = true;
    pkt.flags = payload_buffer_[flags_index];
    {
      uint16_t cmd_le = 0;
      if (!readU16LE(payload_buffer_, cmd_index, cmd_le))
      {
        emitV1();
        return;
      }
      pkt.cmd = cmd_le;
    }
    pkt.payload.assign(payload_buffer_.begin() + header_size, payload_buffer_.begin() + header_size + inner_len);
    if (cb_)
      cb_(pkt);
    stats_.tunneled_v2_rx++;
  }
  MSPPacket MSPParser::makePacketV1() const
  {
    MSPPacket p;
    p.version = MSPVersion::V1;
    p.cmd = v1_command_;
    p.flags = 0;
    p.payload = payload_buffer_;
    p.tunneled = false;
    return p;
  }
  MSPPacket MSPParser::makePacketV2(bool tunneled) const
  {
    MSPPacket p;
    p.version = MSPVersion::V2;
    p.cmd = v2_command_;
    p.flags = v2_flags_;
    p.payload = payload_buffer_;
    p.tunneled = tunneled;
    return p;
  }
  bool MSPParser::abort()
  {
    if (!in_frame_)
      return false;
    stats_.aborted_frames++;
    reset();
    return true;
  }
  void MSPParser::input(uint8_t byte_value)
  {
    // Fast path: seek frame start.
    if (!in_frame_)
    {
      if (byte_value == '$')
      {
        reset();
        in_frame_ = true;
        frame_pos_ = 1; // '$'
        frame_trace_.push_back(byte_value);
      }
      return;
    }

    frame_trace_.push_back(byte_value);
    ++frame_pos_;

    // Version decision at position 2 ('$' already consumed).
    if (frame_pos_ == 2)
    {
      if (byte_value == 'M')
      {
        current_version_ = MSPVersion::V1;
      }
      else if (byte_value == 'X')
      {
        current_version_ = MSPVersion::V2;
      }
      else
      {
        stats_.malformed_header++;
        reset();
      }
      return;
    }

    // Direction / secondary header char at position 3.
    if (frame_pos_ == 3)
    {
      if (byte_value != '<' && byte_value != '>')
      {
        stats_.malformed_header++;
        reset();
      }
      return;
    }
    if (current_version_ == MSPVersion::V1)
    {
      processV1Byte(byte_value);
    }
    else
    {
      processV2Byte(byte_value);
    }
  }

  void MSPParser::processV1Byte(uint8_t byte_value)
  {
    // MSP v1 frame layout after initial 3-byte signature ($ M <):
    //  pos4: payload length (L)
    //  pos5: command id (1 byte)
    //  pos6..(5+L): payload bytes
    //  pos(6+L): XOR checksum of length, command, and payload bytes
    // We already track frame_pos_ as 1-based across the entire frame.

    switch (frame_pos_)
    {
    case 4:
    { // Length byte
      v1_payload_length_ = byte_value;
      v1_checksum_ = byte_value; // checksum starts with length
      payload_buffer_.clear();
      if (v1_payload_length_ > max_payload_size_)
      {
        stats_.oversized_frame_dropped++;
        reset();
        return;
      }
      payload_buffer_.reserve(v1_payload_length_);
      expected_frame_length_ = kV1BaseFrameLen + v1_payload_length_; // final checksum position
      return;
    }
    case 5:
    { // Command byte
      v1_command_ = byte_value;
      v1_checksum_ ^= byte_value;
      return;
    }
    default:
      break;
    }

    // If we know expected length and have reached final byte -> checksum
    if (expected_frame_length_ && frame_pos_ == expected_frame_length_)
    {
      finishFrame(v1_checksum_ == byte_value, false);
      return;
    }

    // Payload region (positions 6 .. 5+payload_length)
    if (frame_pos_ >= 6 && frame_pos_ < expected_frame_length_)
    {
      const size_t idx = frame_pos_ - 6; // zero-based index inside payload
      if (idx < v1_payload_length_)
      {
        payload_buffer_.push_back(byte_value);
        v1_checksum_ ^= byte_value;
        return;
      }
    }

    // Any other situation is inconsistent with computed frame layout -> reset parser state
    reset();
  }

  void MSPParser::processV2Byte(uint8_t byte_value)
  {
    // MSP v2 frame layout after 3-byte signature ($ X <):
    //  pos4: flags
    //  pos5: command id low
    //  pos6: command id high
    //  pos7: payload length low (lenL)
    //  pos8: payload length high (lenH)
    //  pos9..(8+len): payload bytes
    //  final: CRC8 over flags, cmdL, cmdH, lenL, lenH and payload

    switch (frame_pos_)
    {
    case 4: // flags
      v2_flags_ = byte_value;
      v2_crc_computed_ = crc8_dvb_s2::update(0, byte_value);
      return;
    case 5: // cmd low
      v2_command_ = byte_value;
      v2_crc_computed_ = crc8_dvb_s2::update(v2_crc_computed_, byte_value);
      return;
    case 6: // cmd high
      v2_command_ |= (uint16_t)byte_value << 8;
      v2_crc_computed_ = crc8_dvb_s2::update(v2_crc_computed_, byte_value);
      return;
    case 7: // len low
      v2_payload_length_ = byte_value;
      v2_crc_computed_ = crc8_dvb_s2::update(v2_crc_computed_, byte_value);
      return;
    case 8:
    { // len high and payload buffer init
      v2_payload_length_ |= (uint16_t)byte_value << 8;
      v2_crc_computed_ = crc8_dvb_s2::update(v2_crc_computed_, byte_value);
      payload_buffer_.clear();
      if (v2_payload_length_ > max_payload_size_)
      {
        stats_.oversized_frame_dropped++;
        reset();
        return;
      }
      payload_buffer_.reserve(v2_payload_length_);
      expected_frame_length_ = kV2BaseFrameLen + v2_payload_length_;
      return;
    }
    default:
      break;
    }

    // CRC byte at known final position
    if (expected_frame_length_ && frame_pos_ == expected_frame_length_)
    {
      v2_crc_received_ = byte_value;
      finishFrame(v2_crc_computed_ == v2_crc_received_, true);
      return;
    }

    // Payload region
    if (frame_pos_ >= 9 && frame_pos_ < expected_frame_length_)
    {
      const size_t idx = frame_pos_ - 9; // zero-based payload index
      if (idx < v2_payload_length_)
      {
        payload_buffer_.push_back(byte_value);
        v2_crc_computed_ = crc8_dvb_s2::update(v2_crc_computed_, byte_value);
        return;
      }
    }

    // Any other path means inconsistency -> reset
    reset();
  }

} // namespace transformer_msp_bridge
