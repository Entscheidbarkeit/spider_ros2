/**
 * @file lgpio_backend.cpp
 * @brief Implements a software PWM servo backend using the Linux GPIO (lgpio) interface.
 */

#include "transformer_hw_servos/servo_backend.hpp"

#include <lgpio.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace transformer_hw_servos {

namespace {
constexpr int kPrimaryServoChannels = 2;
constexpr unsigned int kDefaultLineGroup = 0;
constexpr int kDisablePulseWidthUs = 0;
constexpr int kDisableOffsetUs = 0;
constexpr int kDisableCycles = 0;
constexpr int kMinRealtimePriority = 1;
constexpr int kMaxRealtimePriority = 99;
constexpr long kMinLogicalCpus = 1L;
constexpr int kUnsetCpuAffinity = -1;
constexpr size_t kBytesPerMegabyte = 1024ULL * 1024ULL;
constexpr int kInvalidHandle = -1;
constexpr char kDigitCharacters[] = "0123456789";
}  // namespace

/**
 * @brief Servo backend that generates PWM signals via the lgpio software interface.
 */
class LgpioServoBackend : public IServoBackend {
 public:
  bool init(const ServoBackendConfig& cfg) override {
    cfg_ = cfg;
    ApplyRealtimeSettings();

    const int chip_index = ParseChipIndex(cfg_.gpiochip_name);
    handle_ = lgGpiochipOpen(chip_index);
    if (handle_ < 0) {
      handle_ = kInvalidHandle;
      return false;
    }

    if (lgGpioClaimOutput(handle_, kDefaultLineGroup, cfg_.gpio_servo[0], kDisablePulseWidthUs) < 0 ||
        lgGpioClaimOutput(handle_, kDefaultLineGroup, cfg_.gpio_servo[1], kDisablePulseWidthUs) < 0) {
      lgGpiochipClose(handle_);
      handle_ = kInvalidHandle;
      return false;
    }

    // Program initial pulse widths so the servos begin in a known state.
    lgTxServo(handle_, cfg_.gpio_servo[0], cfg_.initial_pulse_us[0], cfg_.freq_hz, cfg_.offset_us[0], cfg_.cycles);
    if (cfg_.channels_in_use >= kPrimaryServoChannels) {
      lgTxServo(handle_, cfg_.gpio_servo[1], cfg_.initial_pulse_us[1], cfg_.freq_hz, cfg_.offset_us[1], cfg_.cycles);
    }
    return true;
  }

  bool setPulse(int index, int pulse_us, int offset) override {
    if (handle_ == kInvalidHandle || index < 0 || index >= kPrimaryServoChannels) {
      return false;
    }
    lgTxServo(handle_, cfg_.gpio_servo[index], pulse_us+offset, cfg_.freq_hz, cfg_.offset_us[index], cfg_.cycles);
    return true;
  }

  bool setDualPulse(int pulse0_us, int pulse1_us) override {
    if (handle_ == kInvalidHandle) {
      return false;
    }
    lgTxServo(handle_, cfg_.gpio_servo[0], pulse0_us, cfg_.freq_hz, cfg_.offset_us[0], cfg_.cycles);
    lgTxServo(handle_, cfg_.gpio_servo[1], pulse1_us, cfg_.freq_hz, cfg_.offset_us[1], cfg_.cycles);
    return true;
  }
    bool setAllPulse(int pulse0_us, int pulse1_us, int pulse2_us, int pulse3_us) override{
    const bool ok0 = setPulse(0, pulse0_us,0);
    const bool ok1 = setPulse(1, pulse1_us,0);
    const bool ok2 = setPulse(2, pulse2_us,0);
    const bool ok3 = setPulse(3, pulse3_us,0);
    return ok0 && ok1 && ok2 &&ok3;
  }

  void shutdown() override {
    if (handle_ == kInvalidHandle) {
      return;
    }
    // Stop driving both channels before closing the chip handle.
    lgTxServo(handle_, cfg_.gpio_servo[0], kDisablePulseWidthUs, cfg_.freq_hz, kDisableOffsetUs, kDisableCycles);
    lgTxServo(handle_, cfg_.gpio_servo[1], kDisablePulseWidthUs, cfg_.freq_hz, kDisableOffsetUs, kDisableCycles);
    lgGpiochipClose(handle_);
    handle_ = kInvalidHandle;
  }

  ~LgpioServoBackend() override { shutdown(); }

 private:
  /**
   * @brief Applies realtime kernel scheduling, memory locking, and CPU affinity settings.
   */
  void ApplyRealtimeSettings() {
    if (!cfg_.use_realtime) {
      return;
    }

    if (cfg_.lock_memory) {
      struct rlimit limit {};
      if (getrlimit(RLIMIT_MEMLOCK, &limit) == 0) {
        struct rlimit raised = limit;
        raised.rlim_cur = limit.rlim_max;
        setrlimit(RLIMIT_MEMLOCK, &raised);
      }
      mlockall(MCL_CURRENT | MCL_FUTURE);
      if (cfg_.prefault_mb > 0) {
        const size_t bytes = static_cast<size_t>(cfg_.prefault_mb) * kBytesPerMegabyte;
        prefault_buffer_.assign(bytes, 0);
        const size_t page = static_cast<size_t>(sysconf(_SC_PAGESIZE));
        for (size_t i = 0; i < bytes; i += page) {
          prefault_buffer_[i] = 0;
        }
      }
    }

    struct sched_param scheduler_param {};
    scheduler_param.sched_priority = std::clamp(cfg_.rt_priority, kMinRealtimePriority, kMaxRealtimePriority);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &scheduler_param);

    if (cfg_.cpu_affinity != kUnsetCpuAffinity) {
      cpu_set_t mask;
      CPU_ZERO(&mask);
      const long logical_cpus = std::max(kMinLogicalCpus, sysconf(_SC_NPROCESSORS_ONLN));
      const int cpu = cfg_.cpu_affinity % static_cast<int>(logical_cpus);
      CPU_SET(cpu, &mask);
      pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
    }
  }

  /**
   * @brief Extracts the numeric GPIO chip index from the configured chip name.
   */
  static int ParseChipIndex(const std::string& chip_name) {
    const size_t numeric_start = chip_name.find_last_not_of(kDigitCharacters);
    if (numeric_start == std::string::npos || numeric_start + 1 >= chip_name.size()) {
      return 0;
    }
    const std::string suffix = chip_name.substr(numeric_start + 1);
    try {
      return std::stoi(suffix);
    } catch (...) {
      return 0;
    }
  }

  ServoBackendConfig cfg_{};
  int handle_{kInvalidHandle};
  std::vector<uint8_t> prefault_buffer_{};
};

std::unique_ptr<IServoBackend> create_lgpio_backend() {
  return std::make_unique<LgpioServoBackend>();
}

}  // namespace transformer_hw_servos
