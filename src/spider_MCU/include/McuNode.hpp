#pragma once

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "transformer_mcu_bridge/CommandQueue.hpp"
#include "transformer_mcu_bridge/ProtocolParser.hpp"
#include "transformer_mcu_bridge/SerialManager.hpp"
#include "transformer_mcu_bridge/StateCache.hpp"

namespace MCU
{

class McuNode : public rclcpp::Node
{
public:
  explicit McuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~McuNode() override;

private:
  enum class BridgeLinkState : uint8_t
  {
    kDisconnected = 0,
    kConnecting,
    kHandshaking,
    kReady,
    kError
  };

  void declare_parameters();
  void load_parameters();

  void setup_publishers();
  void setup_subscribers();
  void setup_timers();

  void start_workers();
  void stop_workers();

  void reconnect_timer_callback();
  void poll_timer_callback();
  void publish_timer_callback();

  void raw_cmd_callback(const std_msgs::msg::String::SharedPtr msg);
  void stop_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void servo_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void motor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  void rx_worker_loop();
  void tx_worker_loop();

  void handle_rx_line(const std::string & line);
  void handle_parsed_frame(const ParsedFrame & frame);

  void enqueue_command(
    const std::string & command,
    bool requires_ack = false,
    const std::string & expected_prefix = "",
    std::chrono::milliseconds timeout = std::chrono::milliseconds(500),
    bool high_priority = false);

  bool open_serial();
  void close_serial();

  bool perform_handshake();
  void publish_cached_state();

  [[nodiscard]] std::array<float, 4> adc_to_deg(const std::array<int32_t, 4> & raw) const;

private:
  // Parameters
  std::string port_{"/dev/ttyACM0"};
  int baudrate_{115200};

  int reconnect_period_ms_{1000};
  int poll_period_ms_{100};
  int publish_period_ms_{100};
  int ping_timeout_ms_{300};
  int command_timeout_ms_{500};

  bool poll_adc_{true};
  bool poll_position_{false};
  bool poll_limits_{false};

  // Core modules
  SerialManager serial_manager_{};
  ProtocolParser parser_{};
  CommandQueue command_queue_{};
  StateCache state_cache_{};

  // State
  BridgeLinkState link_state_{BridgeLinkState::kDisconnected};
  bool workers_running_{false};

  // Workers
  std::thread rx_thread_{};
  std::thread tx_thread_{};

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_rx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr adc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pot_deg_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr limit_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr servo_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr motor_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace transformer_mcu_bridge