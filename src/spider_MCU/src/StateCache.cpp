#include "spider_MCU/StateCache.hpp"

namespace MCU
{

void StateCache::set_link_state(const LinkState & link_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.link = link_state;
}

void StateCache::set_adc_state(const AdcState & adc_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.adc = adc_state;
}

void StateCache::set_position_state(const PositionState & position_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.position = position_state;
}

void StateCache::set_limit_state(const LimitState & limit_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.limits = limit_state;
}

void StateCache::set_last_raw_rx(const std::string & line)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.last_raw_rx = line;
}

void StateCache::set_last_ack(const std::string & ack)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.last_ack = ack;
}

void StateCache::set_last_dbg(const std::string & dbg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.last_dbg = dbg;
}

void StateCache::set_last_status_text(const std::string & status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.link.last_status_text = status;
}

void StateCache::set_last_error_text(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.link.last_error_text = error;
}

void StateCache::mark_rx_time(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.link.last_rx_time = stamp;
}

void StateCache::mark_tx_time(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.link.last_tx_time = stamp;
}

void StateCache::mark_ping_time(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.link.last_ping_time = stamp;
}

McuStateSnapshot StateCache::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

}  // namespace MCU