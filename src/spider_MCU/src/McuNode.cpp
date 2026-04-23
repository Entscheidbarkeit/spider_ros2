#include "spider_MCU/McuNode.hpp"

#include <chrono>
#include <utility>
#include <variant>

namespace MCU
{

McuNode::McuNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("spider_mcu_node", options)
{
  declare_parameters();
  load_parameters();

  setup_publishers();
  setup_subscribers();
  setup_timers();
  start_workers();

  RCLCPP_INFO(this->get_logger(), "spider_mcu_node started.");
}

McuNode::~McuNode()
{
  stop_workers();
  close_serial();
}

void McuNode::declare_parameters()
{
  this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  this->declare_parameter<int>("baudrate", 115200);

  this->declare_parameter<int>("reconnect_period_ms", 1000);
  this->declare_parameter<int>("poll_period_ms", 100);
  this->declare_parameter<int>("publish_period_ms", 100);
  this->declare_parameter<int>("ping_timeout_ms", 300);
  this->declare_parameter<int>("command_timeout_ms", 500);

  this->declare_parameter<bool>("poll_adc", true);
  this->declare_parameter<bool>("poll_position", false);
  this->declare_parameter<bool>("poll_limits", false);
}

void McuNode::load_parameters()
{
  port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();

  reconnect_period_ms_ = this->get_parameter("reconnect_period_ms").as_int();
  poll_period_ms_ = this->get_parameter("poll_period_ms").as_int();
  publish_period_ms_ = this->get_parameter("publish_period_ms").as_int();
  ping_timeout_ms_ = this->get_parameter("ping_timeout_ms").as_int();
  command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_int();

  poll_adc_ = this->get_parameter("poll_adc").as_bool();
  poll_position_ = this->get_parameter("poll_position").as_bool();
  poll_limits_ = this->get_parameter("poll_limits").as_bool();
}

void McuNode::setup_publishers()
{
  connected_pub_ = this->create_publisher<std_msgs::msg::Bool>("mcu/connected", 10);
  raw_rx_pub_ = this->create_publisher<std_msgs::msg::String>("mcu/raw_rx", 20);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("mcu/status_text", 20);

  adc_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("mcu/adc", 10);
  pot_deg_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("mcu/pot_deg", 10);
  position_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("mcu/position", 10);
  limit_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("mcu/limit_switch", 10);
}

void McuNode::setup_subscribers()
{
  raw_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mcu/cmd_raw", 20,
    std::bind(&McuNode::raw_cmd_callback, this, std::placeholders::_1));

  stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "mcu/stop", 10,
    std::bind(&McuNode::stop_callback, this, std::placeholders::_1));

  servo_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "mcu/set_servo_pulse", 10,
    std::bind(&McuNode::servo_callback, this, std::placeholders::_1));

  motor_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "mcu/set_motor_percent", 10,
    std::bind(&McuNode::motor_callback, this, std::placeholders::_1));
}

void McuNode::setup_timers()
{
  reconnect_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(reconnect_period_ms_),
    std::bind(&McuNode::reconnect_timer_callback, this));

  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_period_ms_),
    std::bind(&McuNode::poll_timer_callback, this));

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&McuNode::publish_timer_callback, this));
}

void McuNode::start_workers()
{
  if (workers_running_.load()) {
    return;
  }

  workers_running_.store(true);

  rx_thread_ = std::thread(&McuNode::rx_worker_loop, this);
  tx_thread_ = std::thread(&McuNode::tx_worker_loop, this);
}

void McuNode::stop_workers()
{
  if (!workers_running_.load()) {
    return;
  }

  workers_running_.store(false);

  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }

  if (tx_thread_.joinable()) {
    tx_thread_.join();
  }
}

void McuNode::reconnect_timer_callback()
{
  if (serial_manager_.is_connected()) {
    return;
  }

  link_state_ = BridgeLinkState::kConnecting;

  if (!open_serial()) {
    link_state_ = BridgeLinkState::kDisconnected;
    return;
  }

  link_state_ = BridgeLinkState::kHandshaking;

  if (!perform_handshake()) {
    RCLCPP_WARN(this->get_logger(), "Handshake failed.");
    close_serial();
    link_state_ = BridgeLinkState::kError;
    return;
  }

  LinkState link;
  link.serial_open = true;
  link.handshake_ok = true;
  link.connected = true;
  link.port_name = port_;
  link.baudrate = baudrate_;
  link.last_ping_time = this->now();
  state_cache_.set_link_state(link);

  link_state_ = BridgeLinkState::kReady;
  RCLCPP_INFO(this->get_logger(), "Connected to MCU on %s @ %d", port_.c_str(), baudrate_);
}

void McuNode::poll_timer_callback()
{
  if (!serial_manager_.is_connected()) {
    return;
  }

  if (poll_adc_) {
    enqueue_command("ADC", false, "", std::chrono::milliseconds(command_timeout_ms_));
  }

  if (poll_position_) {
    enqueue_command("GETPOS", false, "", std::chrono::milliseconds(command_timeout_ms_));
  }

  if (poll_limits_) {
    enqueue_command("GETLIM", false, "", std::chrono::milliseconds(command_timeout_ms_));
  }
}

void McuNode::publish_timer_callback()
{
  publish_cached_state();
}

void McuNode::raw_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "raw cmd received: %s", msg->data.c_str());
  enqueue_command(msg->data, false, "", std::chrono::milliseconds(command_timeout_ms_));
}

void McuNode::stop_callback(const std_msgs::msg::Empty::SharedPtr)
{
  enqueue_command("STOP", true, "OK", std::chrono::milliseconds(command_timeout_ms_), true);
}

void McuNode::servo_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (!msg || msg->data.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "set_servo_pulse expects [servo_id, pulse_us]");
    return;
  }

  const int servo_id = msg->data[0];
  const int pulse_us = msg->data[1];

  std::string command;
  if (servo_id < 0) {
    command = "SRVALL " + std::to_string(pulse_us);
  } else {
    command = "SRV " + std::to_string(servo_id) + " " + std::to_string(pulse_us);
  }

  enqueue_command(command, true, "OK", std::chrono::milliseconds(command_timeout_ms_));
}

void McuNode::motor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (!msg || msg->data.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "set_motor_percent expects [motor_id, percent]");
    return;
  }

  const int motor_id = msg->data[0];
  const int percent = msg->data[1];

  const std::string command =
    "MOTOR " + std::to_string(motor_id) + " " + std::to_string(percent);

  enqueue_command(command, true, "OK", std::chrono::milliseconds(command_timeout_ms_));
}

void McuNode::rx_worker_loop()
{
  while (workers_running_.load()) {
    if (!serial_manager_.is_connected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    auto line = serial_manager_.read_line(std::chrono::milliseconds(50));
    if (!line.has_value()) {
      continue;
    }

    handle_rx_line(line.value());
  }
}

void McuNode::tx_worker_loop()
{
  while (workers_running_.load()) {
    auto pending = command_queue_.wait_and_pop(std::chrono::milliseconds(100));
    if (!pending.has_value()) {
      continue;
    }

    if (!serial_manager_.is_connected()) {
      continue;
    }
    RCLCPP_INFO(this->get_logger(), "sending: %s", pending->command.c_str());
    if (!serial_manager_.write_line(pending->command)) {
      RCLCPP_WARN(this->get_logger(), "Failed to write command: %s", pending->command.c_str());
      continue;
    }

    state_cache_.mark_tx_time(this->now());
  }
}

void McuNode::handle_rx_line(const std::string & line)
{
  state_cache_.set_last_raw_rx(line);
  state_cache_.mark_rx_time(this->now());

  std_msgs::msg::String raw_msg;
  raw_msg.data = line;
  raw_rx_pub_->publish(raw_msg);

  const ParsedFrame frame = parser_.parse_line(line);
  handle_parsed_frame(frame);
}

void McuNode::handle_parsed_frame(const ParsedFrame & frame)
{
  std::visit(
    [this](auto && parsed) {
      using T = std::decay_t<decltype(parsed)>;

      if constexpr (std::is_same_v<T, OkFrame>) {
        state_cache_.set_last_ack(parsed.text);
        state_cache_.set_last_status_text(parsed.text);

        std_msgs::msg::String msg;
        msg.data = parsed.text;
        status_pub_->publish(msg);
      } else if constexpr (std::is_same_v<T, ErrorFrame>) {
        state_cache_.set_last_error_text(parsed.text);

        std_msgs::msg::String msg;
        msg.data = parsed.text;
        status_pub_->publish(msg);
      } else if constexpr (std::is_same_v<T, PongFrame>) {
        state_cache_.set_last_ack(parsed.text);

        LinkState link = state_cache_.snapshot().link;
        link.serial_open = true;
        link.handshake_ok = true;
        link.connected = true;
        link.port_name = port_;
        link.baudrate = baudrate_;
        link.last_ping_time = this->now();
        state_cache_.set_link_state(link);

        std_msgs::msg::String msg;
        msg.data = parsed.text;
        status_pub_->publish(msg);
      } else if constexpr (std::is_same_v<T, BusyFrame>) {
        std_msgs::msg::String msg;
        msg.data = parsed.text;
        status_pub_->publish(msg);
      } else if constexpr (std::is_same_v<T, ReadyFrame>) {
        std_msgs::msg::String msg;
        msg.data = parsed.text;
        status_pub_->publish(msg);
      } else if constexpr (std::is_same_v<T, AdcFrame>) {
        AdcState adc;
        adc.raw = parsed.values;
        adc.deg = adc_to_deg(parsed.values);
        adc.valid = true;
        state_cache_.set_adc_state(adc);
      } else if constexpr (std::is_same_v<T, PosFrame>) {
        PositionState pos;
        pos.values = parsed.values;
        pos.valid = true;
        state_cache_.set_position_state(pos);
      } else if constexpr (std::is_same_v<T, LimFrame>) {
        LimitState lim;
        lim.values = parsed.values;
        lim.valid = true;
        state_cache_.set_limit_state(lim);
      } else if constexpr (std::is_same_v<T, DbgFrame>) {
        state_cache_.set_last_dbg(parsed.text);

        std_msgs::msg::String msg;
        msg.data = parsed.text;
        status_pub_->publish(msg);
      } else if constexpr (std::is_same_v<T, UnknownFrame>) {
        // keep silent for now
      }
    },
    frame);
}

void McuNode::enqueue_command(
  const std::string & command,
  bool requires_ack,
  const std::string & expected_prefix,
  std::chrono::milliseconds timeout,
  bool high_priority)
{
  static uint64_t sequence_counter = 0;

  PendingCommand pending;
  pending.command = command;
  pending.requires_ack = requires_ack;
  pending.expected_prefix = expected_prefix;
  pending.timeout = timeout;
  pending.high_priority = high_priority;
  pending.sequence_id = ++sequence_counter;
  RCLCPP_INFO(this->get_logger(), "enqueue: %s", command.c_str());
  if (high_priority) {
    command_queue_.push_front(pending);
  } else {
    command_queue_.push(pending);
  }
}

bool McuNode::open_serial()
{
  if (!serial_manager_.connect(port_, static_cast<uint32_t>(baudrate_))) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to open serial port %s @ %d",
      port_.c_str(), baudrate_);
    return false;
  }

  LinkState link = state_cache_.snapshot().link;
  link.serial_open = true;
  link.connected = false;
  link.handshake_ok = false;
  link.port_name = port_;
  link.baudrate = baudrate_;
  state_cache_.set_link_state(link);

  serial_manager_.flush_input();
  serial_manager_.flush_output();

  return true;
}

void McuNode::close_serial()
{
  serial_manager_.disconnect();

  LinkState link = state_cache_.snapshot().link;
  link.serial_open = false;
  link.connected = false;
  link.handshake_ok = false;
  state_cache_.set_link_state(link);

  link_state_ = BridgeLinkState::kDisconnected;
}

bool McuNode::perform_handshake()
{
  serial_manager_.flush_input();

  if (!serial_manager_.write_line("PING")) {
    return false;
  }

  state_cache_.mark_ping_time(this->now());

  const auto line = serial_manager_.read_line(std::chrono::milliseconds(ping_timeout_ms_));
  if (!line.has_value()) {
    return false;
  }

  handle_rx_line(line.value());

  return std::holds_alternative<PongFrame>(parser_.parse_line(line.value()));
}

void McuNode::publish_cached_state()
{
  const McuStateSnapshot snapshot = state_cache_.snapshot();

  std_msgs::msg::Bool connected_msg;
  connected_msg.data = snapshot.link.connected && serial_manager_.is_connected();
  connected_pub_->publish(connected_msg);

  if (snapshot.adc.valid) {
    std_msgs::msg::Int32MultiArray adc_msg;
    adc_msg.data.assign(snapshot.adc.raw.begin(), snapshot.adc.raw.end());
    adc_pub_->publish(adc_msg);

    std_msgs::msg::Float32MultiArray pot_msg;
    pot_msg.data.assign(snapshot.adc.deg.begin(), snapshot.adc.deg.end());
    pot_deg_pub_->publish(pot_msg);
  }

  if (snapshot.position.valid) {
    std_msgs::msg::Int32MultiArray pos_msg;
    pos_msg.data.assign(snapshot.position.values.begin(), snapshot.position.values.end());
    position_pub_->publish(pos_msg);
  }

  if (snapshot.limits.valid) {
    std_msgs::msg::Int32MultiArray lim_msg;
    lim_msg.data.assign(snapshot.limits.values.begin(), snapshot.limits.values.end());
    limit_pub_->publish(lim_msg);
  }
}

std::array<float, 4> McuNode::adc_to_deg(const std::array<int32_t, 4> & raw) const
{
  std::array<float, 4> deg{};

  for (std::size_t i = 0; i < raw.size(); ++i) {
    deg[i] = static_cast<float>(raw[i]) * 360.0F / 4095.0F;
  }

  return deg;
}

}  // namespace MCU