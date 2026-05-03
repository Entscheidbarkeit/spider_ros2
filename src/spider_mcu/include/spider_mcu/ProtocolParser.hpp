#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace MCU
{

enum class FrameType : uint8_t
{
  kUnknown = 0,
  kOk,
  kError,
  kPong,
  kBusy,
  kReady,
  kAdc,
  kPos,
  kLim,
  kDbg
};

struct OkFrame
{
  std::string text{};
};

struct ErrorFrame
{
  std::string text{};
};

struct PongFrame
{
  std::string text{"PONG"};
};

struct BusyFrame
{
  std::string text{"BUSY"};
};

struct ReadyFrame
{
  std::string text{"READY"};
};

struct AdcFrame
{
  std::array<int32_t, 4> values{{0, 0, 0, 0}};
};

struct PosFrame
{
  std::array<int32_t, 4> values{{0, 0, 0, 0}};
};

struct LimFrame
{
  std::array<int32_t, 4> values{{0, 0, 0, 0}};
};

struct DbgFrame
{
  std::string text{};
};

struct UnknownFrame
{
  std::string raw{};
};

using ParsedFrame = std::variant<
  OkFrame,
  ErrorFrame,
  PongFrame,
  BusyFrame,
  ReadyFrame,
  AdcFrame,
  PosFrame,
  LimFrame,
  DbgFrame,
  UnknownFrame>;

class ProtocolParser
{
public:
  ProtocolParser() = default;
  ~ProtocolParser() = default;

  [[nodiscard]] ParsedFrame parse_line(const std::string & line) const;
  [[nodiscard]] FrameType detect_type(const std::string & line) const;

  [[nodiscard]] static std::string trim(const std::string & input);
  [[nodiscard]] static std::vector<std::string> split_ws(const std::string & input);

private:
  [[nodiscard]] std::optional<AdcFrame> parse_adc(const std::vector<std::string> & tokens) const;
  [[nodiscard]] std::optional<PosFrame> parse_pos(const std::vector<std::string> & tokens) const;
  [[nodiscard]] std::optional<LimFrame> parse_lim(const std::vector<std::string> & tokens) const;
};

}  // namespace MCU