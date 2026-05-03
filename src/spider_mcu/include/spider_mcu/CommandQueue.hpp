#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

namespace MCU
{

struct PendingCommand
{
  std::string command{};
  bool requires_ack{false};

  std::string expected_prefix{};
  std::chrono::milliseconds timeout{500};

  bool high_priority{false};
  uint64_t sequence_id{0};
};

class CommandQueue
{
public:
  CommandQueue() = default;
  ~CommandQueue() = default;

  void push(const PendingCommand & cmd);
  void push_front(const PendingCommand & cmd);

  [[nodiscard]] std::optional<PendingCommand> wait_and_pop(std::chrono::milliseconds timeout);
  [[nodiscard]] std::optional<PendingCommand> try_pop();

  void clear();

  [[nodiscard]] bool empty() const;
  [[nodiscard]] std::size_t size() const;

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<PendingCommand> queue_{};
};

}  // namespace MCU