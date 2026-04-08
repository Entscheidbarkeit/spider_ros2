/**
 * @file servos_node.cpp
 * @brief Implements the ServoControllerNode responsible for precise servo motion control.
 */

#include "transformer_hw_servos/servo_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace transformer_hw_servos {

namespace {

constexpr std::string_view kPhaseLinear = "linear";
constexpr std::string_view kPhaseAccel = "accel";
constexpr std::string_view kPhaseCruise = "cruise";
constexpr std::string_view kPhaseDecel = "decel";

constexpr double kHalf = 0.5;
constexpr double kCompleteFraction = 1.0;
constexpr double kPositiveDirection = 1.0;
constexpr double kNegativeDirection = -1.0;



/**
 * @brief Returns a human-readable profile name based on trapezoid configuration.
 * @param use_trapezoid True when a multi-phase profile is active.
 * @param triangular True when acceleration and deceleration phases meet without a cruise segment.
 * @return Descriptive profile label.
 */
std::string DescribeProfile(bool use_trapezoid, bool triangular) {
  if (!use_trapezoid) {
    return "linear";
  }
  return triangular ? "triangular" : "trapezoid";
}

}  // namespace

ServoControllerNode::ServoControllerNode(const rclcpp::NodeOptions& options) : Node("transformer_hw_servos", options) {
  // Parameter loading is done first so later phases can rely on validated configuration values.
  LoadParameters();
  // Clamp to hardware bounds before we propagate the settings to any helper structures.
  ClampParametersToHardwareLimits();
  // Establish the logical pulse cache so we always have a last-known command per channel.
  InitialiseLogicalPulses();
  // Backends depend on the pulse cache and clamped parameters, so instantiate them afterwards.
  InitializeBackend();
  // Push the initial pulse directly to hardware so the actuator state matches the cache.
  ApplyInitialPulses();

  startup_complete_.store(true);
  startup_complete_time_ms_ = NowMs();

  // Defer action advertisement until hardware has had a moment to reach a stable idle state.
  StartAdvertiseTimer();

  RCLCPP_INFO(get_logger(),
              "\033[32m[READY] Servo controller initialised backend=%s freq=%dHz channels=%d advertise_delay=%ldms\033[0m",
              backend_type_.c_str(), frequency_hz_, channels_in_use_, settle_delay_.count());

  rclcpp::on_shutdown([this]() { BeginShutdown(); });
}

ServoControllerNode::~ServoControllerNode() {
  BeginShutdown();
  if (backend_ != nullptr) {
    backend_->shutdown();
  }
}

void ServoControllerNode::LoadParameters() {
  // Backend selection (GPIO vs PWM expander) dictates which hardware implementation we spin up.
  backend_type_ = declare_parameter<std::string>("backend_type", "pca9685");
  // Timing and range parameters mirror the historic defaults and user overrides.
  period_us_ = declare_parameter<int>("period_us", kDefaultPeriodUs);
  min_pulse_us_ = declare_parameter<int>("min_us", 1'350);
  max_pulse_us_ = declare_parameter<int>("max_us", kHardwarePulseMaxUs);
  initial_pulse_us_ = declare_parameter<int>("initial_us", 900);
  channels_in_use_ = declare_parameter<int>("channels_in_use", 1);

  // Mechanical offsets let us compensate per-servo center or linkage bias.
  auto raw_offsets = declare_parameter<std::vector<int64_t>>("position_offsets_us", std::vector<int64_t>{});
  position_offsets_us_.assign(raw_offsets.begin(), raw_offsets.end());

  // Motion-profile defaults configure acceleration heuristics for trapezoidal moves.
  enable_trapezoid_profile_ = declare_parameter<bool>("enable_trapezoid", false);
  configured_accel_us_per_s2_ = declare_parameter<int>("accel_us_per_s2", 0);

  settle_delay_ = kDefaultAdvertiseDelay;

}

void ServoControllerNode::ClampParametersToHardwareLimits() {
  // Channels map to hardware pins, therefore ensure the user cannot exceed the device capacity.
  channels_in_use_ = std::clamp(channels_in_use_, 1, kMaxServoChannels);

  min_pulse_us_ = std::clamp(min_pulse_us_, kHardwarePulseMinUs, kHardwarePulseMaxUs);
  max_pulse_us_ = std::clamp(max_pulse_us_, kHardwarePulseMinUs, kHardwarePulseMaxUs);
  if (min_pulse_us_ > max_pulse_us_) {
    std::swap(min_pulse_us_, max_pulse_us_);
  }

  // Convert the requested period to a frequency, falling back to the feedback loop rate if invalid.
  frequency_hz_ = period_us_ > 0 ? static_cast<int>(1'000'000 / period_us_) : static_cast<int>(kMotionFeedbackHz);
  const int unclamped_frequency = frequency_hz_;
  frequency_hz_ = std::clamp(frequency_hz_, kHardwareFrequencyMinHz, kHardwareFrequencyMaxHz);
  if (frequency_hz_ != unclamped_frequency) {
    RCLCPP_WARN(get_logger(), "Requested frequency %dHz clamped to %dHz", unclamped_frequency, frequency_hz_);
  }

  if (position_offsets_us_.size() < static_cast<std::size_t>(channels_in_use_)) {
    position_offsets_us_.resize(static_cast<std::size_t>(channels_in_use_), 0);
  }
}

void ServoControllerNode::InitialiseLogicalPulses() {
  // Store the logical command each channel should be holding so feedback/result logic stays consistent.
  const int clamped_initial = ClampLogicalPulse(initial_pulse_us_);
  logical_pulses_us_.assign(static_cast<std::size_t>(channels_in_use_), clamped_initial);
}

void ServoControllerNode::InitializeBackend() {
  // Build a backend configuration describing the static capabilities we expect the driver to obey.
  ServoBackendConfig config;
  config.freq_hz = frequency_hz_;
  config.min_us_global = kHardwarePulseMinUs;
  config.max_us_global = kHardwarePulseMaxUs;
  config.channels_in_use = channels_in_use_;

  for (int channel = 0; channel < channels_in_use_; ++channel) {
    // Individual channel limits are still set to the hardware envelope because offsets may expand range.
    config.min_us_per_servo[channel] = kHardwarePulseMinUs;
    config.max_us_per_servo[channel] = kHardwarePulseMaxUs;
    config.offset_us[channel] = 0;
    config.initial_pulse_us[channel] = ClampHardwarePulse(logical_pulses_us_.at(static_cast<std::size_t>(channel)));
  }

  // Select a backend factory based on the declared type, rejecting unsupported strings early.
  if (backend_type_ == "lgpio") {
    backend_ = create_lgpio_backend();
  } else if (backend_type_ == "pca9685") {
    backend_ = create_pca9685_backend();
  } else {
    throw std::runtime_error("Unsupported backend_type: " + backend_type_);
  }

  if (backend_ == nullptr || !backend_->init(config)) {
    throw std::runtime_error("Failed to initialise servo backend");
  }
}

void ServoControllerNode::ApplyInitialPulses() {
  // Immediately program the hardware so that robot joints park at a predictable, user-defined position.
  for (int channel = 0; channel < channels_in_use_; ++channel) {
    const int offset = GetPositionOffset(channel);
    const int physical_pulse = ClampHardwarePulse(logical_pulses_us_.at(static_cast<std::size_t>(channel)) + offset);
    backend_->setPulse(channel, physical_pulse, this->servoOffset.at(channel));
  }
}

void ServoControllerNode::StartAdvertiseTimer() {
  // Polling avoids advertising the action server too early while remaining simple to reason about.
  advertise_timer_ = create_wall_timer(kAdvertisePollInterval, [this]() {
    if (action_server_ != nullptr) {
      return;
    }
    if ((NowMs() - startup_complete_time_ms_) < settle_delay_.count()) {
      return;
    }
    CreateActionServer();
  });
}

void ServoControllerNode::CreateActionServer() {
  if (action_server_ != nullptr) {
    return;
  }

  action_server_ = rclcpp_action::create_server<MoveServo>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "move_servo",
      std::bind(&ServoControllerNode::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ServoControllerNode::HandleCancel, this, std::placeholders::_1),
      std::bind(&ServoControllerNode::HandleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "\033[32m[READY] Servo action server advertised backend=%s\033[0m",
              backend_type_.c_str());
}

void ServoControllerNode::BeginShutdown() {
  bool expected = false;
  if (!shutting_down_.compare_exchange_strong(expected, true)) {
    return;
  }

  // Abort any motion goal first so controllers upstream receive a failure notification promptly.
  RCLCPP_INFO(get_logger(), "Servo controller shutting down: aborting goals and neutralising outputs");

  CancelActiveGoalForShutdown();

  // Drive every channel back to a neutral pulse to leave actuators in a safe state.
  if (backend_ != nullptr) {
    for (int channel = 0; channel < channels_in_use_; ++channel) {
      const int logical_pulse = logical_pulses_us_.size() > static_cast<std::size_t>(channel)
                                    ? logical_pulses_us_.at(static_cast<std::size_t>(channel))
                                    : ClampLogicalPulse(initial_pulse_us_);
      const int physical_pulse = ClampHardwarePulse(logical_pulse + GetPositionOffset(channel));
      backend_->setPulse(channel, physical_pulse, this->servoOffset.at(channel));
    }
  }

  if (worker_thread_.joinable()) {
    if (worker_thread_.get_id() == std::this_thread::get_id()) {
      RCLCPP_WARN(
          get_logger(),
          "Worker thread matches shutdown thread; skipping join to avoid deadlock. Background thread will exit");
    } else {
      worker_thread_.join();
    }
  }

  // At this point the backend has been neutralised and the worker thread has exited safely.
  RCLCPP_INFO(get_logger(), "Servo controller shutdown complete");
}

void ServoControllerNode::CancelActiveGoalForShutdown() {
  std::shared_ptr<GoalHandleMove> goal_handle;
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    goal_handle = active_goal_;
  }

  if (goal_handle == nullptr) {
    return;
  }

  auto result = std::make_shared<MoveServo::Result>();
  result->success = false;
  result->message = "Aborted due to node shutdown";
  goal_handle->abort(result);
  ClearActiveGoal(goal_handle);
}

void ServoControllerNode::ClearActiveGoal(const std::shared_ptr<GoalHandleMove>& goal_handle) {
  std::lock_guard<std::mutex> lock(active_goal_mutex_);
  if (active_goal_ == goal_handle) {
    active_goal_.reset();
  }
}

rclcpp_action::GoalResponse ServoControllerNode::HandleGoal(const rclcpp_action::GoalUUID& uuid,
                                                            std::shared_ptr<const MoveServo::Goal> goal) {
  (void)uuid;

  if (goal == nullptr || goal->channels.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting servo goal: no channels provided");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (shutting_down_.load()) {
    RCLCPP_WARN(get_logger(), "Rejecting servo goal: shutting down");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_ != nullptr) {
      RCLCPP_WARN(get_logger(), "Rejecting servo goal: another goal already active");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  std::vector<int32_t> seen;
  seen.reserve(goal->channels.size());

  for (const auto channel : goal->channels) {
    if (channel < 1 || channel > channels_in_use_) {
      RCLCPP_WARN(get_logger(), "Rejecting servo goal: channel %d out of range [1, %d]", channel, channels_in_use_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (std::find(seen.begin(), seen.end(), channel) != seen.end()) {
      RCLCPP_WARN(get_logger(), "Rejecting servo goal: duplicate channel index %d", channel);
      return rclcpp_action::GoalResponse::REJECT;
    }
    seen.push_back(channel);
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ServoControllerNode::HandleCancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ServoControllerNode::HandleAccepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
  if (shutting_down_.load()) {
    auto result = std::make_shared<MoveServo::Result>();
    result->success = false;
    result->message = "Shutdown in progress";
    goal_handle->abort(result);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_goal_ = goal_handle;
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  worker_thread_ = std::thread(&ServoControllerNode::ExecuteGoal, this, goal_handle);
}

void ServoControllerNode::ExecuteGoal(const std::shared_ptr<GoalHandleMove> goal_handle) {
  if (shutting_down_.load()) {
    AbortGoalWithMessage(goal_handle, "Node shutting down");
    return;
  }

  const std::optional<GoalExecutionData> data = PrepareGoalExecution(goal_handle);
  if (!data.has_value()) {
    return;
  }

  const GoalExecutionData& execution = data.value();

  if (HandleImmediateExecution(goal_handle, execution)) {
    return;
  }

  const MotionExecutionPlan plan = BuildMotionExecutionPlan(execution);

  LogMotionProfileSummary(execution.channel_states.size(),
                          execution.worst_distance,
                          execution.requested_speed,
                          plan.profile_description,
                          plan.estimated_duration);

  if (!ExecuteMotionProfile(goal_handle, execution, plan)) {
    return;
  }

  PublishSuccessResult(goal_handle,
                       execution.channel_states,
                       execution.logical_target,
                       plan.estimated_duration,
                       "completed profile=" + plan.profile_description);
}

std::optional<ServoControllerNode::GoalExecutionData> ServoControllerNode::PrepareGoalExecution(
    const std::shared_ptr<GoalHandleMove>& goal_handle) {
  const auto goal = goal_handle->get_goal();
  if (goal == nullptr) {
    AbortGoalWithMessage(goal_handle, "Goal was null");
    return std::nullopt;
  }

  if (goal->channels.empty()) {
    AbortGoalWithMessage(goal_handle, "Goal contained no channels");
    return std::nullopt;
  }

  GoalExecutionData data;
  data.logical_target = ClampLogicalPulse(goal->pulse_us);
  data.requested_speed = goal->speed_us_per_s;
  data.use_trapezoid = goal->use_trapezoid || (!goal->use_trapezoid && enable_trapezoid_profile_);
  data.requested_accel = goal->accel_us_per_s2 > 0 ? goal->accel_us_per_s2 : configured_accel_us_per_s2_;

  std::vector<int> channel_indices;
  channel_indices.reserve(goal->channels.size());
  for (const auto channel : goal->channels) {
    channel_indices.push_back(channel - 1);
  }

  data.channel_states = BuildChannelStates(channel_indices, data.logical_target, &data.worst_distance);
  if (data.channel_states.empty()) {
    AbortGoalWithMessage(goal_handle, "No valid channels in goal");
    return std::nullopt;
  }

  return data;
}

bool ServoControllerNode::HandleImmediateExecution(const std::shared_ptr<GoalHandleMove>& goal_handle,
                                                   const GoalExecutionData& data) {
  const bool speed_invalid = data.requested_speed <= 0;
  const bool already_at_target = data.worst_distance < kTargetReachedEpsilonUs;
  if (!speed_invalid && !already_at_target) {
    return false;
  }

  const std::string message = already_at_target ? "already at target" : "applied instantly";
  PublishSuccessResult(goal_handle, data.channel_states, data.logical_target, 0.0, message);
  return true;
}

ServoControllerNode::MotionExecutionPlan ServoControllerNode::BuildMotionExecutionPlan(const GoalExecutionData& data) {
  MotionExecutionPlan plan;
  plan.profile.trapezoid = data.use_trapezoid;

  if (!plan.profile.trapezoid) {
    plan.estimated_duration = data.worst_distance / static_cast<double>(data.requested_speed);
    plan.profile_description = DescribeProfile(false, false);
    return plan;
  }

  plan.profile.cruise = static_cast<double>(data.requested_speed);
  plan.profile.accel = data.requested_accel > 0 ? static_cast<double>(data.requested_accel)
                                                : std::max(plan.profile.cruise / kAccelHeuristicTimeSec,
                                                           plan.profile.cruise * kAccelFallbackMultiplier);
  plan.profile.accel_time = plan.profile.cruise / plan.profile.accel;

  const double accel_distance = kHalf * plan.profile.accel * plan.profile.accel_time * plan.profile.accel_time;
  if ((2.0 * accel_distance) >= data.worst_distance) {
    plan.profile.triangular = true;
    plan.profile.accel_time = std::sqrt(data.worst_distance / plan.profile.accel);
    plan.profile.total_time = 2.0 * plan.profile.accel_time;
    plan.profile.cruise = plan.profile.accel * plan.profile.accel_time;
    plan.estimated_duration = plan.profile.total_time;
  } else {
    const double cruise_distance = data.worst_distance - (2.0 * accel_distance);
    plan.profile.cruise_time = cruise_distance / plan.profile.cruise;
    plan.profile.total_time = (2.0 * plan.profile.accel_time) + plan.profile.cruise_time;
    plan.estimated_duration = plan.profile.total_time;
  }

  plan.profile_description = DescribeProfile(true, plan.profile.triangular);
  return plan;
}

bool ServoControllerNode::ExecuteMotionProfile(const std::shared_ptr<GoalHandleMove>& goal_handle,
                                               const GoalExecutionData& data,
                                               const MotionExecutionPlan& plan) {
  auto feedback = std::make_shared<MoveServo::Feedback>();
  feedback->current_pulses_us.resize(data.channel_states.size());

  double elapsed_seconds = 0.0;

  const auto emit_fraction = [&](double fraction, std::string_view phase_label) {
    const double clamped_fraction = std::clamp(fraction, 0.0, kCompleteFraction);

    for (std::size_t index = 0; index < data.channel_states.size(); ++index) {
      const auto& state = data.channel_states[index];
      const double travelled = state.absolute_distance_us * clamped_fraction;
      const double physical_value = state.physical_start_us + (state.direction * travelled);
      const int physical_pulse = ClampHardwarePulse(static_cast<int>(std::round(physical_value)));

      backend_->setPulse(state.channel_index, physical_pulse, this->servoOffset.at(state.channel_index));

      const int logical_value = ClampLogicalPulse(physical_pulse - state.position_offset_us);
      logical_pulses_us_[state.channel_index] = logical_value;
      feedback->current_pulses_us[index] = logical_value;
    }

    feedback->progress = static_cast<float>(clamped_fraction);
    feedback->estimated_remaining_s = static_cast<float>(std::max(0.0, plan.estimated_duration - elapsed_seconds));
    feedback->phase = phase_label;

    goal_handle->publish_feedback(feedback);
  };

  const auto publish_cancel_result = [&](const char* reason) {
    auto result = std::make_shared<MoveServo::Result>();
    result->success = false;
    result->message = reason;
    result->total_estimated_duration_s = static_cast<float>(plan.estimated_duration);
    result->final_pulses_us.resize(data.channel_states.size());
    for (std::size_t index = 0; index < data.channel_states.size(); ++index) {
      const int logical_value = logical_pulses_us_.at(data.channel_states[index].channel_index);
      result->final_pulses_us[index] = logical_value;
    }
    goal_handle->canceled(result);
  };

  if (!plan.profile.trapezoid) {
    return ExecuteLinearProfile(data.worst_distance,
                                data.requested_speed,
                                goal_handle,
                                emit_fraction,
                                publish_cancel_result,
                                elapsed_seconds);
  }

  const auto phase_solver = [&](double elapsed) -> std::optional<std::pair<double, std::string_view>> {
    if (!plan.profile.triangular) {
      if (elapsed < plan.profile.accel_time) {
        const double distance = kHalf * plan.profile.accel * elapsed * elapsed;
        return std::pair{distance / data.worst_distance, kPhaseAccel};
      }

      if (elapsed < (plan.profile.accel_time + plan.profile.cruise_time)) {
        const double accel_distance = kHalf * plan.profile.accel * plan.profile.accel_time * plan.profile.accel_time;
        const double cruise_time = elapsed - plan.profile.accel_time;
        const double distance = accel_distance + (plan.profile.cruise * cruise_time);
        return std::pair{distance / data.worst_distance, kPhaseCruise};
      }

      if (elapsed < plan.profile.total_time) {
        const double accel_distance = kHalf * plan.profile.accel * plan.profile.accel_time * plan.profile.accel_time;
        const double cruise_distance = plan.profile.cruise * plan.profile.cruise_time;
        const double decel_time = elapsed - (plan.profile.accel_time + plan.profile.cruise_time);
        const double decel_distance = (plan.profile.cruise * decel_time) -
                                      (kHalf * plan.profile.accel * decel_time * decel_time);
        const double distance = accel_distance + cruise_distance + decel_distance;
        return std::pair{distance / data.worst_distance, kPhaseDecel};
      }

      return std::nullopt;
    }

    if (elapsed < plan.profile.accel_time) {
      const double distance = kHalf * plan.profile.accel * elapsed * elapsed;
      return std::pair{distance / data.worst_distance, kPhaseAccel};
    }

    if (elapsed < plan.profile.total_time) {
      const double accel_distance = kHalf * plan.profile.accel * plan.profile.accel_time * plan.profile.accel_time;
      const double decel_time = elapsed - plan.profile.accel_time;
      const double peak_velocity = plan.profile.accel * plan.profile.accel_time;
      const double decel_distance = (peak_velocity * decel_time) -
                                    (kHalf * plan.profile.accel * decel_time * decel_time);
      const double distance = accel_distance + decel_distance;
      return std::pair{distance / data.worst_distance, kPhaseDecel};
    }

    return std::nullopt;
  };

  return ExecuteTrapezoidProfile(goal_handle, emit_fraction, publish_cancel_result, elapsed_seconds, phase_solver);
}

int64_t ServoControllerNode::NowMs() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int ServoControllerNode::ClampLogicalPulse(int pulse_us) const {
  return std::clamp(pulse_us, min_pulse_us_, max_pulse_us_);
}

int ServoControllerNode::ClampHardwarePulse(int pulse_us) {
  return std::clamp(pulse_us, kHardwarePulseMinUs, kHardwarePulseMaxUs);
}

int ServoControllerNode::GetPositionOffset(int channel_index) const {
  if (channel_index < 0 || channel_index >= static_cast<int>(position_offsets_us_.size())) {
    return 0;
  }
  return static_cast<int>(position_offsets_us_.at(static_cast<std::size_t>(channel_index)));
}

std::vector<ServoControllerNode::ChannelState> ServoControllerNode::BuildChannelStates(
    const std::vector<int>& channel_indices, int logical_target_us, double* out_worst_distance_us) {
  std::vector<ChannelState> channel_states;
  channel_states.reserve(channel_indices.size());
  double worst_distance = 0.0;

  for (const int index : channel_indices) {
    const int logical_start = logical_pulses_us_.at(static_cast<std::size_t>(index));
    const int offset = GetPositionOffset(index);
    const int physical_start = ClampHardwarePulse(logical_start + offset);
    const int physical_target = ClampHardwarePulse(logical_target_us + offset);
    const double distance = static_cast<double>(physical_target - physical_start);
    const double absolute_distance = std::abs(distance);
    worst_distance = std::max(worst_distance, absolute_distance);

    const double direction = distance >= 0.0 ? kPositiveDirection : kNegativeDirection;

    channel_states.push_back(ChannelState{index,
                                          logical_start,
                                          physical_start,
                                          physical_target,
                                          offset,
                                          absolute_distance,
                                          direction});
  }

  if (out_worst_distance_us != nullptr) {
    *out_worst_distance_us = worst_distance;
  }
  return channel_states;
}

bool ServoControllerNode::ExecuteLinearProfile(double worst_distance,
                                               int requested_speed,
                                               const std::shared_ptr<GoalHandleMove>& goal_handle,
                                               const std::function<void(double, std::string_view)>& emit_fraction,
                                               const std::function<void(const char*)>& publish_cancel_result,
                                               double& elapsed_seconds) {
  rclcpp::Rate rate{std::chrono::duration<double>(kMotionFeedbackPeriodSec)};

  const double step_us = static_cast<double>(requested_speed) / kMotionFeedbackHz;
  double travelled = 0.0;

  while (rclcpp::ok() && travelled < (worst_distance - kTargetReachedEpsilonUs)) {
    if (goal_handle->is_canceling()) {
      publish_cancel_result("canceled");
      ClearActiveGoal(goal_handle);
      return false;
    }

    travelled += std::min(step_us, worst_distance - travelled);

    const double fraction = travelled / worst_distance;
    emit_fraction(fraction, kPhaseLinear);

    rate.sleep();
    elapsed_seconds += kMotionFeedbackPeriodSec;
  }

  return true;
}

bool ServoControllerNode::ExecuteTrapezoidProfile(
    const std::shared_ptr<GoalHandleMove>& goal_handle,
    const std::function<void(double, std::string_view)>& emit_fraction,
    const std::function<void(const char*)>& publish_cancel_result,
    double& elapsed_seconds,
    const std::function<std::optional<std::pair<double, std::string_view>>(double)>& phase_solver) {
  rclcpp::Rate rate{std::chrono::duration<double>(kMotionFeedbackPeriodSec)};

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      publish_cancel_result("canceled");
      ClearActiveGoal(goal_handle);
      return false;
    }

    const std::optional<std::pair<double, std::string_view>> phase_state = phase_solver(elapsed_seconds);
    if (!phase_state.has_value()) {
      break;
    }

    emit_fraction(phase_state->first, phase_state->second);

    if (phase_state->first >= kCompleteFraction) {
      break;
    }

    rate.sleep();
    elapsed_seconds += kMotionFeedbackPeriodSec;
  }

  return true;
}

void ServoControllerNode::AbortGoalWithMessage(const std::shared_ptr<GoalHandleMove>& goal_handle,
                                               const std::string& reason) {
  auto result = std::make_shared<MoveServo::Result>();
  result->success = false;
  result->message = reason;
  goal_handle->abort(result);
  ClearActiveGoal(goal_handle);
}

void ServoControllerNode::PublishSuccessResult(const std::shared_ptr<GoalHandleMove>& goal_handle,
                                               const std::vector<ChannelState>& channel_states, int logical_target_us,
                                               double estimated_duration_s, const std::string& result_message) {
  // Sync internal cache and hardware to the final target before notifying the client of success.
  for (const auto& state : channel_states) {
    backend_->setPulse(state.channel_index, state.physical_target_us, this->servoOffset.at(state.channel_index));
    logical_pulses_us_[state.channel_index] = logical_target_us;
  }

  auto result = std::make_shared<MoveServo::Result>();
  result->success = true;
  result->message = result_message;
  result->final_pulses_us.resize(channel_states.size());
  for (std::size_t index = 0; index < channel_states.size(); ++index) {
    const auto& state = channel_states[index];
    // Convert the hardware pulse back into logical space so consumers do not need to know offsets.
    result->final_pulses_us[index] = ClampLogicalPulse(state.physical_target_us - state.position_offset_us);
  }
  result->total_estimated_duration_s = static_cast<float>(estimated_duration_s);

  goal_handle->succeed(result);
  ClearActiveGoal(goal_handle);
}

void ServoControllerNode::LogMotionProfileSummary(std::size_t channel_count, double worst_distance, int speed,
                                                  const std::string& profile_type, double estimated_duration) {
  RCLCPP_INFO(get_logger(), "Move request channels=%zu max_distance=%.1f speed=%d profile=%s estimated_duration=%.3fs",
              channel_count, worst_distance, speed, profile_type.c_str(), estimated_duration);
}

int RunServoControllerNode(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ServoControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("transformer_hw_servos"), "Fatal error: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

}  // namespace transformer_hw_servos

/**
 * @brief Application entry point delegating to the servo controller node runner.
 */
int main(int argc, char** argv) {
  return transformer_hw_servos::RunServoControllerNode(argc, argv);
}
