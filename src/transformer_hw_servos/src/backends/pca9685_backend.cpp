/**
 * @file pca9685_backend.cpp
 * @brief Implements a register-level PCA9685 PWM servo backend using the Linux i2c-dev interface.
 */

#include "transformer_hw_servos/servo_backend.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

namespace transformer_hw_servos {

namespace {
constexpr uint8_t kRegMode1 = 0x00;
constexpr uint8_t kRegMode2 = 0x01;
constexpr uint8_t kRegLed0OnL = 0x06;
constexpr uint8_t kRegPrescale = 0xFE;

constexpr uint8_t kMode1RestartBit = 0x80;
constexpr uint8_t kMode1SleepBit = 0x10;
constexpr uint8_t kMode1AutoIncrementBit = 0x20;
constexpr uint8_t kMode1ExternalClockBit = 0x40;
constexpr uint8_t kMode2TotemPole = 0x04;
constexpr uint8_t kHighByteMask = 0x0F;

constexpr double kOscillatorFrequencyHz = 25'000'000.0;
constexpr double kPwmResolutionTicks = 4096.0;
constexpr uint16_t kMaxTickValue = 4095;
constexpr uint16_t kTickMask = 0x0FFF;
constexpr std::chrono::milliseconds kDeviceWakeDelay{5};
constexpr double kMicrosecondsPerSecond = 1'000'000.0;

constexpr int kPrimaryServoChannels = 4; 
constexpr int kMaxSupportedChannels = 6;
constexpr int kTotalDeviceChannels = 16;
constexpr int kMinPrescale = 3;
constexpr int kMaxPrescale = 255;
constexpr int kHardwareMinHz = 24;
constexpr int kHardwareMaxHz = 1526;

bool WriteRegister(int fd, uint8_t reg, uint8_t value) {
  const uint8_t buffer[2]{reg, value};
  return (::write(fd, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer)));
}

bool ReadRegister(int fd, uint8_t reg, uint8_t& value) {
  if (::write(fd, &reg, 1) != 1) {
    return false;
  }
  if (::read(fd, &value, 1) != 1) {
    return false;
  }
  return true;
}

bool WriteChannel(int fd, int channel, uint16_t on_count, uint16_t off_count) {
  const uint8_t base = static_cast<uint8_t>(kRegLed0OnL + (4 * channel));
  const uint8_t buffer[5]{base,
                          static_cast<uint8_t>(on_count & 0xFF),
                          static_cast<uint8_t>((on_count >> 8) & kHighByteMask),
                          static_cast<uint8_t>(off_count & 0xFF),
                          static_cast<uint8_t>((off_count >> 8) & kHighByteMask)};
  return (::write(fd, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer)));
}

int ClampPrescaleForFrequency(int requested_frequency_hz) {
  const int clamped_frequency = std::clamp(requested_frequency_hz, kHardwareMinHz, kHardwareMaxHz);
  const double prescale_value = kOscillatorFrequencyHz / (kPwmResolutionTicks * static_cast<double>(clamped_frequency)) - 1.0;
  int prescale = static_cast<int>(std::round(prescale_value));
  prescale = std::clamp(prescale, kMinPrescale, kMaxPrescale);
  return prescale;
}
}  // namespace

/**
 * @brief Servo backend for the PCA9685 hardware PWM controller.
 */
class Pca9685Backend : public IServoBackend {
 public:
  bool init(const ServoBackendConfig& cfg) override {
    cfg_ = cfg;
    fd_ = ::open(cfg_.i2c_bus.c_str(), O_RDWR);
    if (fd_ < 0) {
      return false;
    }
    if (ioctl(fd_, I2C_SLAVE, cfg_.pca9685_address) < 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    // Reset MODE1 to a known baseline with the restart bit cleared.
    WriteRegister(fd_, kRegMode1, 0x00);
    std::this_thread::sleep_for(kDeviceWakeDelay);

    const int prescale = ClampPrescaleForFrequency(cfg_.freq_hz);

    uint8_t previous_mode1 = 0;
    if (!ReadRegister(fd_, kRegMode1, previous_mode1)) {
      return false;
    }
    const uint8_t cleared_restart = static_cast<uint8_t>(previous_mode1 & static_cast<uint8_t>(~kMode1RestartBit));
    const uint8_t sleep_mode = static_cast<uint8_t>((cleared_restart & static_cast<uint8_t>(~kMode1ExternalClockBit)) | kMode1SleepBit);
    WriteRegister(fd_, kRegMode1, sleep_mode);
    WriteRegister(fd_, kRegPrescale, static_cast<uint8_t>(prescale));

    // Wake up the device and enable auto-increment so multi-byte writes advance automatically.
    const uint8_t wake_mode = static_cast<uint8_t>((cleared_restart & static_cast<uint8_t>(~kMode1SleepBit)) | kMode1AutoIncrementBit);
    WriteRegister(fd_, kRegMode1, wake_mode);
    std::this_thread::sleep_for(kDeviceWakeDelay);
    WriteRegister(fd_, kRegMode2, kMode2TotemPole);  // Totem pole, non-inverted, update on STOP.

    for (int index = 0; index < cfg_.channels_in_use && index < kTotalDeviceChannels; ++index) {
      setPulse(index, cfg_.initial_pulse_us[index],0); // initial pulse without offset
    }
    return true;
  }

  bool setPulse(int index, int pulse_us, int offset) override {
    if (fd_ < 0 || index < 0) {
      return false;
    }
    const int hardware_channel = MapChannel(index);
    if (hardware_channel < 0) {
      return false;
    }

    const double period_us = kMicrosecondsPerSecond / static_cast<double>(cfg_.freq_hz);
    if (period_us <= 0.0) {
      return false;
    }

    const double ticks_per_us = kPwmResolutionTicks / period_us;
    const int offset_us = cfg_.offset_us[index];
    const int on_count = static_cast<int>(std::fmod(offset_us * ticks_per_us, kPwmResolutionTicks));
    int pulse_ticks = static_cast<int>((pulse_us + offset) * ticks_per_us + 0.5);
    pulse_ticks = std::clamp(pulse_ticks, 0, static_cast<int>(kMaxTickValue));
    const int off_count = (on_count + pulse_ticks) & kTickMask;
    return WriteChannel(fd_, hardware_channel, static_cast<uint16_t>(on_count), static_cast<uint16_t>(off_count));
  }

  bool setDualPulse(int pulse0_us, int pulse1_us) override { // this function is possible deprecated, the offset here are dummy params
    const bool ok0 = setPulse(0, pulse0_us, 0);
    const bool ok1 = setPulse(1, pulse1_us, 0);
    return ok0 && ok1;
  }
  bool setAllPulse(int pulse0_us, int pulse1_us, int pulse2_us, int pulse3_us) override{ // this function is possible deprecated, the offset here are dummy params
    const bool ok0 = setPulse(0, pulse0_us,0);
    const bool ok1 = setPulse(1, pulse1_us,0);
    const bool ok2 = setPulse(2, pulse2_us,0);
    const bool ok3 = setPulse(3, pulse3_us,0);
    return ok0 && ok1 && ok2 &&ok3;
  }

  void shutdown() override {
    if (fd_ < 0) {
      return;
    }
    for (int index = 0; index < kPrimaryServoChannels; ++index) {
      const int channel = cfg_.pca9685_channel[index];
      if (channel >= 0 && channel < kTotalDeviceChannels) {
        WriteChannel(fd_, channel, 0, 0);
      }
    }
    ::close(fd_);
    fd_ = -1;
  }

  ~Pca9685Backend() override { shutdown(); }

 private:
  int MapChannel(int index) const {
    if (index < kPrimaryServoChannels) {
      return cfg_.pca9685_channel[index];
    }
    if (index < kMaxSupportedChannels) {
      const int base = std::max(cfg_.pca9685_channel[0], cfg_.pca9685_channel[1]) + 1;
      return base + (index - kPrimaryServoChannels);
    }
    return -1;
  }

  ServoBackendConfig cfg_{};
  int fd_{-1};
};

std::unique_ptr<IServoBackend> create_pca9685_backend() {
  return std::make_unique<Pca9685Backend>();
}

}  // namespace transformer_hw_servos
