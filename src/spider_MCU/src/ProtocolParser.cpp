#include "spider_MCU/ProtocolParser.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace MCU
{

ParsedFrame ProtocolParser::parse_line(const std::string & line) const
{
  const std::string cleaned = trim(line);
  if (cleaned.empty()) {
    return UnknownFrame{line};
  }

  const FrameType type = detect_type(cleaned);
  const std::vector<std::string> tokens = split_ws(cleaned);

  switch (type) {
    case FrameType::kOk:
      return OkFrame{cleaned};

    case FrameType::kError:
      return ErrorFrame{cleaned};

    case FrameType::kPong:
      return PongFrame{};

    case FrameType::kBusy:
      return BusyFrame{};

    case FrameType::kReady:
      return ReadyFrame{};

    case FrameType::kAdc: {
      auto parsed = parse_adc(tokens);
      if (parsed.has_value()) {
        return parsed.value();
      }
      return UnknownFrame{line};
    }

    case FrameType::kPos: {
      auto parsed = parse_pos(tokens);
      if (parsed.has_value()) {
        return parsed.value();
      }
      return UnknownFrame{line};
    }

    case FrameType::kLim: {
      auto parsed = parse_lim(tokens);
      if (parsed.has_value()) {
        return parsed.value();
      }
      return UnknownFrame{line};
    }

    case FrameType::kDbg: {
      if (cleaned.size() <= 3) {
        return DbgFrame{""};
      }
      return DbgFrame{trim(cleaned.substr(3))};
    }

    case FrameType::kUnknown:
    default:
      return UnknownFrame{line};
  }
}

FrameType ProtocolParser::detect_type(const std::string & line) const
{
  const std::string cleaned = trim(line);
  if (cleaned.empty()) {
    return FrameType::kUnknown;
  }

  if (cleaned == "PONG") {
    return FrameType::kPong;
  }
  if (cleaned == "BUSY") {
    return FrameType::kBusy;
  }
  if (cleaned == "READY") {
    return FrameType::kReady;
  }

  if (cleaned.rfind("OK", 0) == 0) {
    return FrameType::kOk;
  }
  if (cleaned.rfind("ERR", 0) == 0) {
    return FrameType::kError;
  }
  if (cleaned.rfind("ADC", 0) == 0) {
    return FrameType::kAdc;
  }
  if (cleaned.rfind("POS", 0) == 0) {
    return FrameType::kPos;
  }
  if (cleaned.rfind("LIM", 0) == 0) {
    return FrameType::kLim;
  }
  if (cleaned.rfind("DBG", 0) == 0) {
    return FrameType::kDbg;
  }

  return FrameType::kUnknown;
}

std::string ProtocolParser::trim(const std::string & input)
{
  const auto begin = std::find_if_not(
    input.begin(), input.end(),
    [](unsigned char ch) { return std::isspace(ch) != 0; });

  if (begin == input.end()) {
    return "";
  }

  const auto end = std::find_if_not(
    input.rbegin(), input.rend(),
    [](unsigned char ch) { return std::isspace(ch) != 0; }).base();

  return std::string(begin, end);
}

std::vector<std::string> ProtocolParser::split_ws(const std::string & input)
{
  std::vector<std::string> tokens;
  std::istringstream iss(input);
  std::string token;

  while (iss >> token) {
    tokens.push_back(token);
  }

  return tokens;
}

std::optional<AdcFrame> ProtocolParser::parse_adc(const std::vector<std::string> & tokens) const
{
  if (tokens.size() != 5 || tokens[0] != "ADC") {
    return std::nullopt;
  }

  try {
    AdcFrame frame;
    frame.values[0] = std::stoi(tokens[1]);
    frame.values[1] = std::stoi(tokens[2]);
    frame.values[2] = std::stoi(tokens[3]);
    frame.values[3] = std::stoi(tokens[4]);
    return frame;
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<PosFrame> ProtocolParser::parse_pos(const std::vector<std::string> & tokens) const
{
  if (tokens.size() != 5 || tokens[0] != "POS") {
    return std::nullopt;
  }

  try {
    PosFrame frame;
    frame.values[0] = std::stoi(tokens[1]);
    frame.values[1] = std::stoi(tokens[2]);
    frame.values[2] = std::stoi(tokens[3]);
    frame.values[3] = std::stoi(tokens[4]);
    return frame;
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<LimFrame> ProtocolParser::parse_lim(const std::vector<std::string> & tokens) const
{
  if (tokens.size() != 5 || tokens[0] != "LIM") {
    return std::nullopt;
  }

  try {
    LimFrame frame;
    frame.values[0] = std::stoi(tokens[1]);
    frame.values[1] = std::stoi(tokens[2]);
    frame.values[2] = std::stoi(tokens[3]);
    frame.values[3] = std::stoi(tokens[4]);
    return frame;
  } catch (...) {
    return std::nullopt;
  }
}

}  // namespace MCU