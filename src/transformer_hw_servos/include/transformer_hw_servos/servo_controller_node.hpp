/**
 * @file servo_controller_node.hpp
 * @brief Declares the ServoControllerNode responsible for coordinated servo motion control.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "transformer_hw_servos/action/move_servo.hpp"
#include "transformer_hw_servos/servo_backend.hpp"

namespace transformer_hw_servos {

/**
 * @brief Hardware-safe minimum pulse width supported by attached servos (microseconds).
 */
inline constexpr int kHardwarePulseMinUs = 500;

/**
 * @brief Hardware-safe maximum pulse width supported by attached servos (microseconds).
 */
inline constexpr int kHardwarePulseMaxUs = 2500;

/**
 * @brief Minimum update frequency (Hz) supported by the target servo controller hardware.
 */
inline constexpr int kHardwareFrequencyMinHz = 40;

/**
 * @brief Maximum update frequency (Hz) supported by the target servo controller hardware.
 */
inline constexpr int kHardwareFrequencyMaxHz = 400;

/**
 * @brief Default PWM period in microseconds (equivalent to 50 Hz).
 */
inline constexpr int kDefaultPeriodUs = 20'000;

/**
 * @brief Default settle delay before advertising the action server.
 */
inline constexpr std::chrono::milliseconds kDefaultAdvertiseDelay{1'000};

/**
 * @brief Interval used to poll readiness for advertising the action server.
 */
inline constexpr std::chrono::milliseconds kAdvertisePollInterval{50};

/**
 * @brief Fixed upper bound on servo channels supported by the hardware backend.
 */
inline constexpr int kMaxServoChannels = 16;

/**
 * @brief Feedback loop rate for executing motion profiles (Hz).
 */
inline constexpr double kMotionFeedbackHz = 50.0;

/**
 * @brief Inverse of the motion feedback rate (seconds per loop iteration).
 */
inline constexpr double kMotionFeedbackPeriodSec = 1.0 / kMotionFeedbackHz;

/**
 * @brief Threshold for considering a servo already at its requested position (microseconds).
 */
inline constexpr double kTargetReachedEpsilonUs = 0.5;

/**
 * @brief Heuristic denominator to derive acceleration from the requested cruise speed.
 */
inline constexpr double kAccelHeuristicTimeSec = 0.2;

/**
 * @brief Heuristic multiplier applied when deriving a fallback acceleration.
 */
inline constexpr double kAccelFallbackMultiplier = 5.0;

/**
 * @brief ROS 2 node that exposes servo motion via an action interface with optional motion profiles.
 */
class ServoControllerNode : public rclcpp::Node {
 public:
  using MoveServo = transformer_hw_servos::action::MoveServo;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveServo>;

  /**
   * @brief Constructs the servo controller node and performs all required hardware initialisation.
   * @param options Node options forwarded by the executable or launch description.
   */
  explicit ServoControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Ensures hardware shutdown and goal cancellation on destruction.
   */
  ~ServoControllerNode() override;

 private:
  // ===== Lifecycle helpers =====

  /**
   * @brief Loads configuration parameters from the ROS parameter server.
   */
  void LoadParameters();

  /**
   * @brief Clamps parameter-derived values to hardware safe limits.
   */
  void ClampParametersToHardwareLimits();

  /**
   * @brief Instantiates and initialises the configured servo backend.
   */
  void InitializeBackend();

  /**
   * @brief Applies initial pulse outputs, taking static offsets into account.
   */
  void ApplyInitialPulses();

  /**
   * @brief Starts the timer that defers action server advertisement to allow hardware settle time.
   */
  void StartAdvertiseTimer();

  /**
   * @brief Creates the action server once the settle delay has elapsed.
   */
  void CreateActionServer();

  /**
   * @brief Begins a graceful shutdown sequence, stopping motion and cancelling active goals.
   */
  void BeginShutdown();

  /**
   * @brief Cancels any active action goal during shutdown.
   */
  void CancelActiveGoalForShutdown();

  /**
   * @brief Clears the active goal pointer if it matches the supplied handle.
   * @param goal_handle Candidate goal handle to compare with the tracked active goal.
   */
  void ClearActiveGoal(const std::shared_ptr<GoalHandleMove>& goal_handle);

  // ===== Action callbacks =====

  /**
   * @brief Validates incoming action goals before accepting execution.
   * @param uuid Unique identifier of the incoming goal.
   * @param goal Goal data containing target channels and pulse information.
   * @return GoalResponse deciding whether the goal should be executed.
   */
  rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const MoveServo::Goal> goal);

  /**
   * @brief Accepts cancellation for active goals.
   * @param goal_handle Handle to the goal that should be cancelled.
   * @return Always accepts cancellation requests.
   */
  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleMove> goal_handle);

  /**
   * @brief Spawns a worker thread that executes the accepted goal.
   * @param goal_handle Handle for the newly accepted goal.
   */
  void HandleAccepted(const std::shared_ptr<GoalHandleMove> goal_handle);

  /**
   * @brief Executes a servo motion goal and provides feedback to clients.
   * @param goal_handle Handle exposing goal data and result channels.
   */
  void ExecuteGoal(const std::shared_ptr<GoalHandleMove> goal_handle);

  // ===== Utility helpers =====

  /**
   * @brief Returns the current time in milliseconds relative to steady clock epoch.
   * @return Millisecond timestamp.
   */
  int64_t NowMs() const;

  /**
   * @brief Clamps a logical (user-facing) pulse width to configured bounds.
   * @param pulse_us Requested pulse width in microseconds.
   * @return Clamped pulse width within logical limits.
   */
  int ClampLogicalPulse(int pulse_us) const;

  /**
   * @brief Clamps a physical (hardware-facing) pulse to absolute hardware limits.
   * @param pulse_us Requested pulse width in microseconds.
   * @return Clamped pulse width within hardware limits.
   */
  static int ClampHardwarePulse(int pulse_us);

  /**
   * @brief Retrieves the configured position offset for a channel, if any.
   * @param channel_index Zero-based channel index.
   * @return Offset in microseconds that should be added to logical pulses.
   */
  int GetPositionOffset(int channel_index) const;

  /**
   * @brief Helper struct describing the state of an individual servo channel for a motion goal.
   */
  struct ChannelState {
    int channel_index = 0;
    int logical_start_us = 0;
    int physical_start_us = 0;
    int physical_target_us = 0;
    int position_offset_us = 0;
    double absolute_distance_us = 0.0;
    double direction = 1.0;
  };

  struct GoalExecutionData {
    int logical_target = 0;
    int requested_speed = 0;
    bool use_trapezoid = false;
    int requested_accel = 0;
    double worst_distance = 0.0;
    std::vector<ChannelState> channel_states;
  };

  struct MotionProfile {
    bool trapezoid = false;
    bool triangular = false;
    double accel = 0.0;
    double cruise = 0.0;
    double accel_time = 0.0;
    double cruise_time = 0.0;
    double total_time = 0.0;
  };

  struct MotionExecutionPlan {
    MotionProfile profile{};
    double estimated_duration = 0.0;
    std::string profile_description;
  };

  /**
   * @brief Creates ChannelState entries for all channels targeted by the goal.
   * @param channel_indices Zero-based channels requested by the goal.
   * @param logical_target_us Logical pulse requested by the client.
   * @param out_worst_distance_us Longest travel distance that any channel needs to cover.
   * @return Vector containing per-channel state information.
   */
  std::vector<ChannelState> BuildChannelStates(const std::vector<int>& channel_indices,
                                               int logical_target_us,
                                               double* out_worst_distance_us);

  /**
   * @brief Publishes final goal results and clears the active goal.
   * @param goal_handle Action goal handle.
   * @param channel_states Final physical target states for each channel.
   * @param logical_target_us Logical pulse recorded for the channels.
   * @param estimated_duration_s Estimated duration of the executed motion.
   * @param profile_description Human-readable description of the motion profile.
   */
  void PublishSuccessResult(const std::shared_ptr<GoalHandleMove>& goal_handle,
                            const std::vector<ChannelState>& channel_states,
                            int logical_target_us,
                            double estimated_duration_s,
                            const std::string& result_message);

  /**
   * @brief Applies clamped logical pulses to the internal cache for all configured channels.
   */
  void InitialiseLogicalPulses();

  /**
   * @brief Logs a summary of the constructed motion profile.
   * @param channel_count Number of channels covered by the goal.
   * @param worst_distance Longest distance travelled by any channel.
   * @param speed Speed requested in microseconds per second.
   * @param profile_type Description of the active profile (linear, triangular, trapezoid).
   * @param estimated_duration Estimated runtime of the motion.
   */
  void LogMotionProfileSummary(std::size_t channel_count,
                               double worst_distance,
                               int speed,
                               const std::string& profile_type,
                               double estimated_duration);

  std::optional<GoalExecutionData> PrepareGoalExecution(const std::shared_ptr<GoalHandleMove>& goal_handle);

  bool HandleImmediateExecution(const std::shared_ptr<GoalHandleMove>& goal_handle, const GoalExecutionData& data);

  MotionExecutionPlan BuildMotionExecutionPlan(const GoalExecutionData& data);

  bool ExecuteMotionProfile(const std::shared_ptr<GoalHandleMove>& goal_handle,
                            const GoalExecutionData& data,
                            const MotionExecutionPlan& plan);

  /**
   * @brief Executes a linear (constant velocity) motion profile.
   */
  bool ExecuteLinearProfile(double worst_distance,
                            int requested_speed,
                            const std::shared_ptr<GoalHandleMove>& goal_handle,
                            const std::function<void(double, std::string_view)>& emit_fraction,
                            const std::function<void(const char*)>& publish_cancel_result,
                            double& elapsed_seconds);

  /**
   * @brief Executes a trapezoidal (or triangular) motion profile.
   */
  bool ExecuteTrapezoidProfile(const std::shared_ptr<GoalHandleMove>& goal_handle,
                               const std::function<void(double, std::string_view)>& emit_fraction,
                               const std::function<void(const char*)>& publish_cancel_result,
                               double& elapsed_seconds,
                               const std::function<std::optional<std::pair<double, std::string_view>>(double)>& phase_solver);

  void AbortGoalWithMessage(const std::shared_ptr<GoalHandleMove>& goal_handle, const std::string& reason);

  // ===== Data members =====

  std::string backend_type_;
  const std::vector<int> servoOffset = {-30,20,50,20,0,0,0,0,0,0,0,0,0,0,0,0};
  int period_us_ = kDefaultPeriodUs;
  int frequency_hz_ = 0;
  int min_pulse_us_ = 0;
  int max_pulse_us_ = 0;
  int initial_pulse_us_ = 0;
  int channels_in_use_ = 1;
  std::vector<int> logical_pulses_us_;
  bool enable_trapezoid_profile_ = false;
  int configured_accel_us_per_s2_ = 0;
  std::vector<int64_t> position_offsets_us_;

  std::unique_ptr<IServoBackend> backend_;
  rclcpp_action::Server<MoveServo>::SharedPtr action_server_;
  std::chrono::milliseconds settle_delay_ = kDefaultAdvertiseDelay;
  rclcpp::TimerBase::SharedPtr advertise_timer_;

  std::atomic<bool> shutting_down_{false};
  std::mutex active_goal_mutex_;
  std::shared_ptr<GoalHandleMove> active_goal_;
  std::thread worker_thread_;

  std::atomic<bool> startup_complete_{false};
  int64_t startup_complete_time_ms_ = 0;
};

/**
 * @brief Runs the servo controller node within a rclcpp executor until shutdown.
 * @param argc Command line argument count.
 * @param argv Command line argument vector.
 * @return Zero on success; non-zero when initialisation fails.
 */
int RunServoControllerNode(int argc, char** argv);

}  // namespace transformer_hw_servos