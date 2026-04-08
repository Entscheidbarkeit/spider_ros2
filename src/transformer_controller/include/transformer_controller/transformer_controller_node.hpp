/**
 * @file transformer_controller_node.hpp
 * @brief Declaration of the TransformerControllerNode responsible for drive/flight mode orchestration.
 */

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "transformer_controller/action/transform_mode.hpp"
#include "transformer_hw_actuators/action/move_actuator.hpp"
#include "transformer_hw_servos/action/move_servo.hpp"

namespace transformer_controller {
/** @brief Main ROS 2 node coordinating actuator and servo motions for transformer modes. */
class TransformerControllerNode : public rclcpp::Node {
 public:
  using MoveActuator = transformer_hw_actuators::action::MoveActuator;
  using MoveActuatorHandle = rclcpp_action::ClientGoalHandle<MoveActuator>;
  using MoveServo = transformer_hw_servos::action::MoveServo;
  using MoveServoHandle = rclcpp_action::ClientGoalHandle<MoveServo>;
  using TransformMode = transformer_controller::action::TransformMode;
  using TransformHandle = rclcpp_action::ServerGoalHandle<TransformMode>;

  /** @brief Construct the node and declare runtime parameters. */
  TransformerControllerNode();

  /** @brief Ensure worker threads shut down before destruction. */
  ~TransformerControllerNode() override;

 private:
  /** @brief Enumerates supported controller states. */
  enum class Mode { kUnknown, kInit, kIdle, kDrive, kFlight };

  /** @brief Runtime configuration derived from ROS parameters. */
  struct ControllerConfig {
    int retract_speed_percent = kDefaultRetractSpeedPercent;
    int retract_time_ms = kDefaultRetractTimeMs;
    int extend_speed_percent = kDefaultExtendSpeedPercent;
    int extend_time_ms = kDefaultExtendTimeMs;
    int flight_servos_pulse = kDefaultFlightServosPulse;
    int drive_servos_pulse = kDefaultDriveServosPulse;
    int servo_move_speed_us_per_s = kDefaultServoMoveSpeed;
  };

  // === Initialization Flow ===

  /** @brief Load configurable parameters from the ROS parameter server. */
  void LoadParameters();

  /** @brief Periodically poll action servers until they are available. */
  void StartInitializationTick();

  /** @brief Execute the baseline retract sequence executed at startup. */
  void RunStartupRetract();

  /** @brief Publish the transform action server once initialization completes. */
  void AdvertiseTransformServer();

  // === Transform Action Callbacks ===

  /**
   * @brief Validate an incoming goal before accepting it.
   * @param uuid Unique identifier supplied by the action server for this goal.
   * @param goal Requested transform mode payload from the caller.
   * @return Decision describing whether execution should proceed.
   */
  rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const TransformMode::Goal> goal);

  /**
   * @brief Cancel handler for transform goals.
   * @param goal_handle Goal instance invoking the cancellation.
   * @return Whether cancellation is accepted.
   */
  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<TransformHandle>& goal_handle);

  /**
   * @brief Spawn the execution worker when a goal is accepted.
   * @param goal_handle Accepted goal to execute.
   */
  void HandleAccepted(const std::shared_ptr<TransformHandle> goal_handle);

  /**
   * @brief Execute the transform workflow on a background thread.
   * @param goal_handle Goal to process until completion.
   */
  void ExecuteTransform(const std::shared_ptr<TransformHandle> goal_handle);

  // === Motion Helpers ===

  /**
   * @brief Wrapper to synchronously command actuator movement.
   * @param speed_percent Desired actuator speed percentage.
   * @param duration_ms Movement duration in milliseconds.
   * @return True when the action server reports success.
   */
  [[nodiscard]] bool RunActuatorTimedGoal(int speed_percent, int duration_ms);

  /**
   * @brief Retract both actuators using configured parameters.
   * @return True when the retract action succeeds.
   */
  [[nodiscard]] bool RetractActuators();

  /**
   * @brief Extend both actuators using configured parameters.
   * @return True when the extend action succeeds.
   */
  [[nodiscard]] bool ExtendActuators();

  /**
   * @brief Move the wing servos to the requested pulse width.
   * @param pulse Target PWM pulse width in microseconds.
   * @return True when the servo action reports success.
   */
  [[nodiscard]] bool MoveServos(int pulse);

  /**
   * @brief Await a future with polling to avoid blocking spinning threads.
   * \tparam FutureT Future-like type returned by action calls.
   * \tparam Rep Rep type for the supplied timeout duration.
   * \tparam Period Period type for the supplied timeout duration.
   * @param fut Future instance to monitor.
   * @param timeout Maximum time to wait for readiness.
   * @return True if the future becomes ready before the timeout expires.
   */
  template <typename FutureT, typename Rep, typename Period>
  [[nodiscard]] bool WaitForFuture(FutureT& fut, const std::chrono::duration<Rep, Period>& timeout);

  // === Shutdown Coordination ===

  /** @brief Stop outstanding work and join worker threads. */
  void BeginShutdown();

  /** @brief Abort the active goal when shutting down. */
  void CancelActiveGoalForShutdown();

  /** @brief Clear the current active goal pointer under lock. */
  void ClearActiveGoal();

  // === Mode Utilities ===

  /**
   * @brief Record the controller mode.
   * @param mode New controller mode to persist.
   */
  void SetCurrentMode(Mode mode);

  /**
   * @brief Retrieve the controller mode.
   * @return Current controller mode value.
   */
  [[nodiscard]] Mode GetCurrentMode() const;

  /**
   * @brief Convert a mode into a string for logging/results.
   * @param mode Mode to stringify.
   * @return String view referencing the mode label.
   */
  [[nodiscard]] std::string_view ModeToString(Mode mode) const;

  /**
   * @brief Translate textual goal input into a Mode.
   * @param raw_mode Raw mode string from the action goal.
   * @param parsed_mode Output storage for the resolved mode.
   * @return True if the mode string is recognised.
   */
  [[nodiscard]] bool TryParseGoalMode(const std::string& raw_mode, Mode* parsed_mode) const;

  /**
   * @brief Indicate whether a mode allows transformations.
   * @param mode Mode to evaluate for transform eligibility.
   * @return True when transforms are permitted.
   */
  [[nodiscard]] bool IsTransformable(Mode mode) const;

  // Identifiers published on the ROS graph.
  inline static constexpr std::string_view kNodeName = "transformer_controller";
  inline static constexpr std::string_view kTransformActionName = "transform_mode";
  inline static constexpr std::string_view kActuatorActionName = "move_actuator";
  inline static constexpr std::string_view kServoActionName = "move_servo";

  // Canonical string labels for state tracking.
  inline static constexpr std::string_view kDriveModeName = "drive";
  inline static constexpr std::string_view kFlightModeName = "flight";
  inline static constexpr std::string_view kInitModeName = "INIT";
  inline static constexpr std::string_view kIdleModeName = "IDLE";
  inline static constexpr std::string_view kUnknownModeName = "unknown";

  // Timing constants that coordinate polling and action execution.
  inline static constexpr std::chrono::milliseconds kStartupTimerPeriod{200};
  inline static constexpr std::chrono::milliseconds kThrottleInterval{5000};
  inline static constexpr std::chrono::seconds kActionServerWaitTimeout{3};
  inline static constexpr std::chrono::milliseconds kActionServerPollInterval{50};
  inline static constexpr std::chrono::seconds kGoalHandleTimeout{2};
  inline static constexpr std::chrono::seconds kServoResultTimeout{10};
  inline static constexpr std::chrono::milliseconds kResultGracePeriod{3000};
  inline static constexpr std::chrono::milliseconds kRetractRetryDelay{500};
  inline static constexpr std::chrono::milliseconds kFutureCheckInterval{20};
  inline static constexpr std::chrono::milliseconds kFutureSleepInterval{5};

  struct FeedbackPhase {
    std::string_view phase;
    float progress;
  };

  inline static constexpr FeedbackPhase kFeedbackRetracting{"retracting_actuators", 0.2F};
  inline static constexpr FeedbackPhase kFeedbackMovingDrive{"moving_servos_drive", 0.6F};
  inline static constexpr FeedbackPhase kFeedbackMovingFlight{"moving_servos_flight", 0.4F};
  inline static constexpr FeedbackPhase kFeedbackExtending{"extending_actuators", 0.7F};
  inline static constexpr FeedbackPhase kFeedbackDone{"done", 1.0F};

  // Default parameter values mirrored in the launch configuration.
  inline static constexpr int kDefaultRetractSpeedPercent = -100;
  inline static constexpr int kDefaultRetractTimeMs = 5000;
  inline static constexpr int kDefaultExtendSpeedPercent = 100;
  inline static constexpr int kDefaultExtendTimeMs = 4000;
  inline static constexpr int kDefaultFlightServosPulse = 2400;
  inline static constexpr int kDefaultDriveServosPulse = 450;
  inline static constexpr int kDefaultServoMoveSpeed = 8000;
  inline static constexpr int kDefaultServoAccel = 1000;

  // Shared channel mappings used when building action goals.
  inline static constexpr std::array<int, 2> kActuatorIds{0, 1};
  inline static constexpr std::array<int, 4> kServoChannels{1, 2,3,4};

  ControllerConfig config_{};

  std::atomic<bool> init_complete_{false};
  std::atomic<bool> init_failed_{false};
  std::atomic<bool> retract_started_{false};
  std::atomic<bool> startup_motion_active_{false};
  std::atomic<bool> shutting_down_{false};

  rclcpp_action::Client<MoveActuator>::SharedPtr actuator_client_;
  rclcpp_action::Client<MoveServo>::SharedPtr servo_client_;
  rclcpp_action::Server<TransformMode>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::thread worker_thread_;
  std::thread startup_thread_;

  mutable std::mutex active_mutex_;
  std::shared_ptr<TransformHandle> active_transform_goal_;

  mutable std::mutex mode_mutex_;
  Mode current_mode_{Mode::kUnknown};
};

template <typename FutureT, typename Rep, typename Period>
bool TransformerControllerNode::WaitForFuture(FutureT& fut, const std::chrono::duration<Rep, Period>& timeout) {
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    // Poll with a short timeout so executor threads keep processing other callbacks.
    if (fut.wait_for(kFutureCheckInterval) == std::future_status::ready) {
      return true;
    }
    // Bail out once the caller-provided timeout elapses to avoid deadlock.
    if (std::chrono::steady_clock::now() - start > timeout) {
      return false;
    }
    // Prevent busy-waiting while still checking regularly for completion.
    std::this_thread::sleep_for(kFutureSleepInterval);
  }
  return false;
}

}  // namespace transformer_controller