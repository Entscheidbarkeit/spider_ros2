#include "transformer_controller/transformer_controller_node.hpp"

#include <utility>

namespace transformer_controller {

TransformerControllerNode::TransformerControllerNode() : Node(std::string(kNodeName)) {
  // Pull configuration from the parameter server before we touch any hardware interfaces.
  LoadParameters();

  // Instantiate action clients used for actuators and servos; we wait for readiness later.
  actuator_client_ = rclcpp_action::create_client<MoveActuator>(this, std::string(kActuatorActionName));
  servo_client_ = rclcpp_action::create_client<MoveServo>(this, std::string(kServoActionName));

  RCLCPP_INFO(get_logger(),
              "TransformerController entering INIT state; waiting for action servers before retracting actuators.");

  // INIT state persists until the retract completes; ensures transforms are rejected beforehand.
  SetCurrentMode(Mode::kInit);

  // Periodic timer checks action servers until they are ready.
  startup_timer_ = create_wall_timer(kStartupTimerPeriod, [this]() { StartInitializationTick(); });

  // Register cleanup callback so external shutdown signals drain worker threads cleanly.
  rclcpp::on_shutdown([this]() { BeginShutdown(); });
}

TransformerControllerNode::~TransformerControllerNode() {
  // Mirror the shutdown callback so explicit destruction also drains work safely.
  BeginShutdown();
}

void TransformerControllerNode::LoadParameters() {
  // Seed runtime configuration with defaults so missing parameters do not stall startup.
  config_.retract_speed_percent = declare_parameter<int>("retract_speed_percent", kDefaultRetractSpeedPercent);
  config_.retract_time_ms = declare_parameter<int>("retract_time_ms", kDefaultRetractTimeMs);
  config_.extend_speed_percent = declare_parameter<int>("extend_speed_percent", kDefaultExtendSpeedPercent);
  config_.extend_time_ms = declare_parameter<int>("extend_time_ms", kDefaultExtendTimeMs);
  config_.flight_servos_pulse = declare_parameter<int>("flight_servos_pulse", kDefaultFlightServosPulse);
  config_.drive_servos_pulse = declare_parameter<int>("drive_servos_pulse", kDefaultDriveServosPulse);
  config_.servo_move_speed_us_per_s = declare_parameter<int>("servo_move_speed_us_per_s", kDefaultServoMoveSpeed);
}

void TransformerControllerNode::StartInitializationTick() {
  if (init_complete_.load() || init_failed_.load() || shutting_down_.load()) {
    return;
  }

  // Query both action servers; transformation support requires both subsystems.
  const bool actuator_ready = actuator_client_ && actuator_client_->action_server_is_ready();
  const bool servo_ready = servo_client_ && servo_client_->action_server_is_ready();

  if (!actuator_ready || !servo_ready) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleInterval.count(),
                         "[INIT] Waiting for action servers (actuator=%s servo=%s)",
                         actuator_ready ? "ready" : "waiting", servo_ready ? "ready" : "waiting");
    return;
  }

  bool expected = false;
  if (!retract_started_.compare_exchange_strong(expected, true)) {
    return;
  }

  RCLCPP_INFO(get_logger(), "[INIT] Action servers ready. Retracting actuators (%d ms @ %d%%).",
              config_.retract_time_ms, config_.retract_speed_percent);

  // Spawn retract thread to keep the timer responsive during the long-running motion.
  startup_motion_active_.store(true);
  startup_thread_ = std::thread(&TransformerControllerNode::RunStartupRetract, this);
}

void TransformerControllerNode::RunStartupRetract() {
  // Helper ensures the motion-active flag always clears even on early returns.
  auto clear_motion_flag = [this]() { startup_motion_active_.store(false); };

  bool success = RetractActuators();
  clear_motion_flag();

  if (!success) {
    RCLCPP_ERROR(get_logger(), "[INIT] Retract failed; retry once after %ld ms", kRetractRetryDelay.count());
    std::this_thread::sleep_for(kRetractRetryDelay);

    if (shutting_down_.load()) {
      // If shutdown happens between attempts, exit silently so shutdown logic owns the result.
      return;
    }

    startup_motion_active_.store(true);
    success = RetractActuators();
    clear_motion_flag();

    if (!success) {
      RCLCPP_FATAL(get_logger(), "[INIT] Retract retry failed; initialization aborted");
      init_failed_.store(true);
      return;
    }

    RCLCPP_WARN(get_logger(), "[INIT] Retract succeeded on retry");
  }

  SetCurrentMode(Mode::kIdle);
  init_complete_.store(true);
  RCLCPP_INFO(get_logger(), "\033[32m[READY] Initialization complete. Mode=IDLE (actuators retracted)\033[0m");

  if (startup_timer_) {
    startup_timer_->cancel();
  }

  AdvertiseTransformServer();
}

void TransformerControllerNode::AdvertiseTransformServer() {
  if (action_server_) {
    return;
  }

  // Goal, cancel, and execution callbacks are bound separately to keep logic testable and focused.
  action_server_ = rclcpp_action::create_server<TransformMode>(
      this, std::string(kTransformActionName),
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const TransformMode::Goal> goal) {
        return HandleGoal(uuid, goal);
      },
      [this](const std::shared_ptr<TransformHandle>& goal_handle) { return HandleCancel(goal_handle); },
      [this](const std::shared_ptr<TransformHandle>& goal_handle) { HandleAccepted(goal_handle); });

  RCLCPP_INFO(get_logger(), "\033[32m[READY] Transform action server advertised\033[0m");
}

rclcpp_action::GoalResponse TransformerControllerNode::HandleGoal(const rclcpp_action::GoalUUID&,
                                                                  std::shared_ptr<const TransformMode::Goal> goal) {
  if (!goal) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!init_complete_.load()) {
    // Still retracting; safe to refuse so we do not compete with startup motions.
    RCLCPP_WARN(get_logger(), "Rejecting transform goal while initialization incomplete");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (startup_motion_active_.load()) {
    RCLCPP_WARN(get_logger(), "Rejecting transform goal while startup motion active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (active_transform_goal_) {
      // Avoid interleaving actuator/servo commands by serialising goal execution.
      RCLCPP_WARN(get_logger(), "Rejecting transform goal: another transform already active");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  Mode requested_mode = Mode::kUnknown;
  if (!TryParseGoalMode(goal->target_mode, &requested_mode)) {
    RCLCPP_WARN(get_logger(), "Rejecting transform goal with unsupported target '%s'", goal->target_mode.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  const Mode current_mode = GetCurrentMode();
  if (requested_mode == current_mode) {
    RCLCPP_INFO(get_logger(), "Rejecting transform goal: already in requested mode %s",
                ModeToString(current_mode).data());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!IsTransformable(current_mode)) {
    // Guard against unexpected intermediate states; only idle/drive/flight may request transitions.
    RCLCPP_WARN(get_logger(), "Rejecting transform goal: current mode %s is not transformable",
                ModeToString(current_mode).data());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TransformerControllerNode::HandleCancel(const std::shared_ptr<TransformHandle>&) {
  // We always allow cancellation; the worker thread routinely checks for goal validity.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TransformerControllerNode::HandleAccepted(const std::shared_ptr<TransformHandle> goal_handle) {
  if (!goal_handle) {
    return;
  }

  if (shutting_down_.load()) {
    auto result = std::make_shared<TransformMode::Result>();
    result->success = false;
    result->message = "shutdown in progress";
    goal_handle->abort(result);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (active_transform_goal_) {
      // Another thread raced us; abort the newcomer so the first goal can finish.
      auto result = std::make_shared<TransformMode::Result>();
      result->success = false;
      result->message = "another transform already active";
      goal_handle->abort(result);
      return;
    }
    active_transform_goal_ = goal_handle;
  }

  if (worker_thread_.joinable()) {
    // Join any trailing worker before spawning a fresh one; we expect only one in flight at a time.
    worker_thread_.join();
  }

  worker_thread_ = std::thread(&TransformerControllerNode::ExecuteTransform, this, goal_handle);
}

void TransformerControllerNode::ExecuteTransform(const std::shared_ptr<TransformHandle> goal_handle) {
  struct ScopedGoalCleanup {
    TransformerControllerNode* node;
    ~ScopedGoalCleanup() {
      if (node != nullptr) {
        // Guarantee the active goal slot clears even on early exits.
        node->ClearActiveGoal();
      }
    }
  } goal_guard{this};

  if (!goal_handle) {
    return;
  }

  if (shutting_down_.load()) {
    auto result = std::make_shared<TransformMode::Result>();
    result->success = false;
    result->message = "node shutting down";
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();
  if (!goal) {
    // Caller cancelled before we pulled the payload; abort to keep action contract.
    auto result = std::make_shared<TransformMode::Result>();
    result->success = false;
    result->message = "goal missing";
    goal_handle->abort(result);
    return;
  }

  auto feedback = std::make_shared<TransformMode::Feedback>();
  auto result = std::make_shared<TransformMode::Result>();

  Mode requested_mode = Mode::kUnknown;
  if (!TryParseGoalMode(goal->target_mode, &requested_mode)) {
    result->success = false;
    result->message = "unsupported target mode";
    goal_handle->abort(result);
    return;
  }

  const Mode starting_mode = GetCurrentMode();
  RCLCPP_INFO(get_logger(), "Transform start: %s -> %s", ModeToString(starting_mode).data(),
              ModeToString(requested_mode).data());

  if (!init_complete_.load()) {
    RCLCPP_ERROR(get_logger(), "Execute called before initialization completed; aborting goal");
    result->success = false;
    result->message = "initialization incomplete";
    goal_handle->abort(result);
    return;
  }

  const auto publish_feedback = [&goal_handle, &feedback](const FeedbackPhase& phase) {
    // Broadcast progress snapshots so clients can surface intermediate UI updates.
    feedback->phase = phase.phase;
    feedback->progress = phase.progress;
    goal_handle->publish_feedback(feedback);
  };

  if (requested_mode == Mode::kDrive) {
    if (starting_mode == Mode::kFlight) {
      publish_feedback(kFeedbackRetracting);
      if (!RetractActuators()) {
        result->success = false;
        result->message = "actuator retract failed";
        goal_handle->abort(result);
        return;
      }
    }

    publish_feedback(kFeedbackMovingDrive);
    if (!MoveServos(config_.drive_servos_pulse)) {
      result->success = false;
      result->message = "servo move failed";
      goal_handle->abort(result);
      return;
    }

    SetCurrentMode(Mode::kDrive);
  } else {
    publish_feedback(kFeedbackMovingFlight);
    if (!MoveServos(config_.flight_servos_pulse)) {
      result->success = false;
      result->message = "servo move failed";
      goal_handle->abort(result);
      return;
    }

    publish_feedback(kFeedbackExtending);
    if (!ExtendActuators()) {
      result->success = false;
      result->message = "actuator extend failed";
      goal_handle->abort(result);
      return;
    }

    SetCurrentMode(Mode::kFlight);
  }

  publish_feedback(kFeedbackDone);

  const Mode final_mode = GetCurrentMode();
  result->success = true;
  result->message = "completed";
  result->final_mode = std::string(ModeToString(final_mode));
  goal_handle->succeed(result);

  RCLCPP_INFO(get_logger(), "Transform complete: now %s", ModeToString(final_mode).data());
}

bool TransformerControllerNode::RunActuatorTimedGoal(int speed_percent, int duration_ms) {
  if (!actuator_client_) {
    RCLCPP_ERROR(get_logger(), "Actuator client not initialised");
    return false;
  }

  if (duration_ms <= 0) {
    RCLCPP_ERROR(get_logger(), "Invalid actuator duration %d", duration_ms);
    return false;
  }

  const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(kActionServerWaitTimeout).count();

  if (!actuator_client_->action_server_is_ready()) {
    // Wait for the server to appear, but only within a bounded timeout to avoid hanging.
    RCLCPP_WARN(get_logger(), "Actuator client not yet ready; waiting up to %ld ms before sending goal", wait_ms);

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !actuator_client_->action_server_is_ready()) {
      if (std::chrono::steady_clock::now() - start > kActionServerWaitTimeout) {
        RCLCPP_ERROR(get_logger(), "Actuator action server not ready after wait; aborting timed run");
        return false;
      }
      std::this_thread::sleep_for(kActionServerPollInterval);
    }

    if (!actuator_client_->action_server_is_ready()) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Actuator client became ready after wait");
  }

  MoveActuator::Goal goal;
  goal.actuators.insert(goal.actuators.end(), kActuatorIds.begin(), kActuatorIds.end());
  goal.speed_percents.assign(kActuatorIds.size(), speed_percent);
  goal.duration_ms.assign(kActuatorIds.size(), duration_ms);

  // Build symmetric actuator goals so both cylinders travel together.
  RCLCPP_INFO(get_logger(), "Actuator timed run: speed=%d duration_ms=%d", speed_percent, duration_ms);

  auto goal_future = actuator_client_->async_send_goal(goal);
  if (!WaitForFuture(goal_future, kGoalHandleTimeout)) {
    RCLCPP_ERROR(get_logger(), "Timeout waiting for actuator goal handle (speed=%d duration=%d)", speed_percent,
                 duration_ms);
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Null actuator goal handle (speed=%d duration=%d)", speed_percent, duration_ms);
    return false;
  }

  auto result_future = actuator_client_->async_get_result(goal_handle);
  const auto timeout = std::chrono::milliseconds(duration_ms) + kResultGracePeriod;
  if (!WaitForFuture(result_future, timeout)) {
    RCLCPP_ERROR(get_logger(), "Timeout waiting for actuator result (duration=%d speed=%d)", duration_ms,
                 speed_percent);
    return false;
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
    const std::string message = result.result ? result.result->message : "no result";
    RCLCPP_ERROR(get_logger(), "Actuator run failed: %s", message.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "Actuator timed run success (speed=%d duration_ms=%d)", speed_percent, duration_ms);
  return true;
}

bool TransformerControllerNode::RetractActuators() {
  // Convenience wrapper ensures the retract call always pairs both cylinders.
  return RunActuatorTimedGoal(config_.retract_speed_percent, config_.retract_time_ms);
}

bool TransformerControllerNode::ExtendActuators() {
  // Mirrors retract helper but drives the actuators outward.
  return RunActuatorTimedGoal(config_.extend_speed_percent, config_.extend_time_ms);
}

bool TransformerControllerNode::MoveServos(int pulse) {
  if (!servo_client_) {
    RCLCPP_ERROR(get_logger(), "Servo client not initialised");
    return false;
  }

  // Goal mirrors the dual-servo hardware layout and honours configured slew limits.
  MoveServo::Goal goal;
  goal.channels.insert(goal.channels.end(), kServoChannels.begin(), kServoChannels.end());
  goal.pulse_us = pulse;
  goal.speed_us_per_s = config_.servo_move_speed_us_per_s;
  goal.use_trapezoid = true;
  goal.accel_us_per_s2 = kDefaultServoAccel;

  // Servo controller expects a pre-configured channel map; goal mirrors that shape.
  auto goal_future = servo_client_->async_send_goal(goal);
  if (!WaitForFuture(goal_future, kGoalHandleTimeout)) {
    RCLCPP_ERROR(get_logger(), "Failed to send servo goal (timeout)");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Null servo goal handle");
    return false;
  }

  auto result_future = servo_client_->async_get_result(goal_handle);
  if (!WaitForFuture(result_future, kServoResultTimeout)) {
    RCLCPP_ERROR(get_logger(), "Servo result wait timeout");
    return false;
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
    const std::string message = result.result ? result.result->message : "no result";
    RCLCPP_ERROR(get_logger(), "Servo move failed: %s", message.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "Servo move successfully");
  return true;
}

void TransformerControllerNode::BeginShutdown() {
  bool expected = false;
  if (!shutting_down_.compare_exchange_strong(expected, true)) {
    // Another thread is already handling shutdown; nothing else to do here.
    return;
  }

  RCLCPP_INFO(get_logger(), "Controller beginning shutdown: aborting active transform goal");
  CancelActiveGoalForShutdown();

  if (worker_thread_.joinable()) {
    if (worker_thread_.get_id() == std::this_thread::get_id()) {
      RCLCPP_WARN(get_logger(),
                  "Worker thread matches current thread during shutdown; skipping join to avoid deadlock");
    } else {
      worker_thread_.join();
    }
  }

  if (startup_thread_.joinable()) {
    if (startup_thread_.get_id() == std::this_thread::get_id()) {
      RCLCPP_WARN(get_logger(), "Startup thread matches current thread during shutdown; skipping join");
    } else {
      startup_thread_.join();
    }
  }

  RCLCPP_INFO(get_logger(), "Controller shutdown complete");
}

void TransformerControllerNode::CancelActiveGoalForShutdown() {
  std::shared_ptr<TransformHandle> goal_handle;
  {
    // Snapshot the active goal so we can complete work outside the mutex.
    std::lock_guard<std::mutex> lock(active_mutex_);
    goal_handle = active_transform_goal_;
  }

  if (goal_handle) {
    auto result = std::make_shared<TransformMode::Result>();
    result->success = false;
    result->message = "aborted due to shutdown";
    goal_handle->abort(result);
  }

  ClearActiveGoal();
}

void TransformerControllerNode::ClearActiveGoal() {
  // Reset under lock so HandleGoal observes consistent state.
  std::lock_guard<std::mutex> lock(active_mutex_);
  active_transform_goal_.reset();
}

void TransformerControllerNode::SetCurrentMode(Mode mode) {
  // Mode changes may originate from worker threads; guard to keep readers safe.
  std::lock_guard<std::mutex> lock(mode_mutex_);
  current_mode_ = mode;
}

TransformerControllerNode::Mode TransformerControllerNode::GetCurrentMode() const {
  // Provide a snapshot of mode state for callers making decisions off-thread.
  std::lock_guard<std::mutex> lock(mode_mutex_);
  return current_mode_;
}

std::string_view TransformerControllerNode::ModeToString(Mode mode) const {
  // Centralise string labels so logs and action messages stay consistent.
  switch (mode) {
    case Mode::kInit:
      return kInitModeName;
    case Mode::kIdle:
      return kIdleModeName;
    case Mode::kDrive:
      return kDriveModeName;
    case Mode::kFlight:
      return kFlightModeName;
    case Mode::kUnknown:
    default:
      return kUnknownModeName;
  }
}

bool TransformerControllerNode::TryParseGoalMode(const std::string& raw_mode, Mode* parsed_mode) const {
  if (parsed_mode == nullptr) {
    return false;
  }

  // Only accept canonical strings; anything else is rejected upstream.
  if (raw_mode == kDriveModeName) {
    *parsed_mode = Mode::kDrive;
    return true;
  }

  if (raw_mode == kFlightModeName) {
    *parsed_mode = Mode::kFlight;
    return true;
  }

  *parsed_mode = Mode::kUnknown;
  return false;
}

bool TransformerControllerNode::IsTransformable(Mode mode) const {
  // Restrict transforms to steady-state modes; INIT transitions must complete first.
  return mode == Mode::kIdle || mode == Mode::kDrive || mode == Mode::kFlight;
}

}  // namespace transformer_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<transformer_controller::TransformerControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}