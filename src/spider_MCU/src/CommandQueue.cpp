#include "spider_MCU/CommandQueue.hpp"

namespace MCU{
    void CommandQueue::push(const PendingCommand & cmd){    
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_back(cmd);
        }
        cv_.notify_one();
    }
    void CommandQueue::push_front(const PendingCommand & cmd){
    {        
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_front(cmd);
    }
        cv_.notify_one();
    }
    std::optional<PendingCommand> CommandQueue::try_pop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }

        PendingCommand cmd = queue_.front();
        queue_.pop_front();
        return cmd;
    }
    std::optional<PendingCommand> CommandQueue::wait_and_pop(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        const bool ready = cv_.wait_for(lock, timeout, [this]() {
            return !queue_.empty();
        });

        if (!ready) {
            return std::nullopt;
        }

        PendingCommand cmd = queue_.front();
        queue_.pop_front();
        return cmd;
    }
    void CommandQueue::clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }
    bool CommandQueue::empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    std::size_t CommandQueue::size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
}