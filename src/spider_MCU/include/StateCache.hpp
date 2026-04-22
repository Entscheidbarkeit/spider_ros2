#pragma once

#include <array>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace MCU
{

struct AdcState
{
  std::array<int32_t, 4> raw{{0, 0, 0, 0}};
  std::array<float, 4> deg{{0.0F, 0.0F, 0.0F, 0.0F}};
  bool valid{false};
};

struct PositionState
{
  std::array<int32_t, 4> values{{0, 0, 0, 0}};
  bool valid{false};
};

struct LimitState
{
  std::array<int32_t, 4> values{{0, 0, 0, 0}};
  bool valid{false};
};

struct LinkState
{
  bool serial_open{false};
  bool handshake_ok{false};
  bool connected{false};

  std::string port_name{};
  int baudrate{115200};

  rclcpp::Time last_rx_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tx_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ping_time{0, 0, RCL_ROS_TIME};

  std::string last_status_text{};
  std::string last_error_text{};
};

struct McuStateSnapshot
{
  LinkState link{};
  AdcState adc{};
  PositionState position{};
  LimitState limits{};

  std::string last_raw_rx{};
  std::string last_ack{};
  std::string last_dbg{};
};

class StateCache
{
public:
  StateCache() = default;
  ~StateCache() = default;

  void set_link_state(const LinkState & link_state);
  void set_adc_state(const AdcState & adc_state);
  void set_position_state(const PositionState & position_state);
  void set_limit_state(const LimitState & limit_state);

  void set_last_raw_rx(const std::string & line);
  void set_last_ack(const std::string & ack);
  void set_last_dbg(const std::string & dbg);
  void set_last_status_text(const std::string & status);
  void set_last_error_text(const std::string & error);

  void mark_rx_time(const rclcpp::Time & stamp);
  void mark_tx_time(const rclcpp::Time & stamp);
  void mark_ping_time(const rclcpp::Time & stamp);

  [[nodiscard]] McuStateSnapshot snapshot() const;

private:
  mutable std::mutex mutex_;
  McuStateSnapshot state_{};
};

}  // namespace transformer_mcu_bridge