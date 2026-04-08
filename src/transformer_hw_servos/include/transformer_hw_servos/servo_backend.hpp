// Abstraction layer for interchangeable servo control backends
// Supports (1) software PWM using lgpio (existing implementation) and
// (2) PCA9685 I2C PWM expander.

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace transformer_hw_servos {

struct ServoBackendConfig {
  int freq_hz{50};          // Servo PWM frequency
  int min_us_global{500};   // Global min bound (already clamped by node logic)
  int max_us_global{2500};  // Global max bound
  int min_us_per_servo[16]{500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500};
  int max_us_per_servo[16]{2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500,
                           2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
  int offset_us[16]{0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0};  // Phase offsets per channel (not position offsets)
  int cycles{0};                              // 0=infinite (lgpio backend only)
  int initial_pulse_us[16]{900, 900, 900, 900, 900, 900, 900, 900,
                           900, 900, 900, 900, 900, 900, 900, 900, };
  // lgpio specific
  std::string gpiochip_name{"gpiochip4"};
  int gpio_servo[2]{6, 18};
  // PCA9685 specific
  std::string i2c_bus{"/dev/i2c-1"};
  int pca9685_address{0x40};
  int pca9685_channel[4]{0,1,2,3};  // channels used for logical servo 0 and 1 (others derived sequentially)
  int channels_in_use{4};        // number of logical channels driven (<=16)
  // Realtime (used only by software PWM backend)
  bool use_realtime{false};
  int rt_priority{80};
  bool lock_memory{true};
  int cpu_affinity{-1};
  int prefault_mb{0};
};

class IServoBackend {
 public:
  virtual ~IServoBackend() = default;
  virtual bool init(const ServoBackendConfig& cfg) = 0;
  virtual bool setPulse(int index, int pulse_us, int offset) = 0;           // set one channel
  virtual bool setDualPulse(int pulse0_us, int pulse1_us) = 0;  // update both as tight as backend allows
  virtual bool setAllPulse(int pulse0_us, int pulse1_us, int pulse2_us,int pulse3_us) = 0; // update all servo
  virtual void shutdown() = 0;                                  // graceful stop
};

// Factory helpers
std::unique_ptr<IServoBackend> create_lgpio_backend();
std::unique_ptr<IServoBackend> create_pca9685_backend();

}  // namespace transformer_hw_servos
