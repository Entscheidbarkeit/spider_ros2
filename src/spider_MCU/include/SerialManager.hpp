#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace MCU
{

class SerialManager
{
public:
  SerialManager();
  ~SerialManager();

  SerialManager(const SerialManager &) = delete;
  SerialManager & operator=(const SerialManager &) = delete;

  bool connect(const std::string & port, uint32_t baudrate);
  void disconnect();

  [[nodiscard]] bool is_connected() const;
  [[nodiscard]] std::string port() const;
  [[nodiscard]] uint32_t baudrate() const;

  bool write_line(const std::string & line);
  bool write_bytes(const std::string & bytes);

  [[nodiscard]] std::optional<std::string> read_line(std::chrono::milliseconds timeout);
  [[nodiscard]] std::size_t available() const;

  void flush_input();
  void flush_output();

private:
  bool open_port_locked(const std::string & port, uint32_t baudrate);
  void close_port_locked();

private:
  mutable std::mutex mutex_;

  int fd_{-1};
  std::atomic<bool> connected_{false};

  std::string port_{};
  uint32_t baudrate_{115200};

  std::string rx_buffer_{};
};

}  // namespace transformer_mcu_bridge