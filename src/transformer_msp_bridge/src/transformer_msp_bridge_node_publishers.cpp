//该节点单独定义了该节点的public的各类函数
#include "transformer_msp_bridge/transformer_msp_bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace transformer_msp_bridge
{

void TransformerMspBridgeNode::configurePublishers()
{
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/msp/imu", rclcpp::SensorDataQoS());
  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/msp/mag", rclcpp::SensorDataQoS());
  attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/msp/attitude", rclcpp::QoS(10));
  altitude_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/altitude", rclcpp::QoS(10));
  vertical_speed_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/msp/vertical_speed", rclcpp::QoS(10));
  baro_altitude_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/barometer_altitude", rclcpp::QoS(10));
  gps_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/msp/gps/fix", rclcpp::QoS(10));
  gps_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/msp/gps/vel", rclcpp::QoS(10));
  gps_home_vec_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/msp/home/vector", rclcpp::QoS(10));
  gps_home_dist_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/home/distance", rclcpp::QoS(10));
  rc_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/msp/rc", rclcpp::QoS(10));
  servo_pub_ = create_publisher<sensor_msgs::msg::JointState>("/msp/servo", rclcpp::QoS(10));
  motor_pub_ = create_publisher<sensor_msgs::msg::JointState>("/msp/motor", rclcpp::QoS(10));
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/msp/battery", rclcpp::QoS(10));
  inav_status_pub_ = create_publisher<transformer_msp_bridge::msg::MspInavStatus>("/msp/inav_status", rclcpp::QoS(10));
  inav_generic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/msp/inav_generic", rclcpp::QoS(10));
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/msp/status_ex", rclcpp::QoS(10));
  sensor_status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/msp/sensors/status", rclcpp::QoS(10));
  gps_stats_status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/msp/gps/statistics", rclcpp::QoS(10));
  gps_stats_array_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/msp/gps/statistics_array", rclcpp::QoS(10));
  rc_tuning_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/msp/rc/tuning", rclcpp::QoS(10));
  rtc_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/msp/rtc", rclcpp::QoS(10));
}

void TransformerMspBridgeNode::publishImu(const ImuSample &sample)
{
  if (imu_pub_)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_imu_;
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;
    msg.linear_acceleration.x = sample.linear_acceleration_mps2[0];
    msg.linear_acceleration.y = sample.linear_acceleration_mps2[1];
    msg.linear_acceleration.z = sample.linear_acceleration_mps2[2];
    msg.angular_velocity.x = sample.angular_velocity_radps[0];
    msg.angular_velocity.y = sample.angular_velocity_radps[1];
    msg.angular_velocity.z = sample.angular_velocity_radps[2];
    imu_pub_->publish(msg);
  }

  if (mag_pub_)
  {
    sensor_msgs::msg::MagneticField mag;
    mag.header.stamp = now();
    mag.header.frame_id = frame_id_mag_;
    mag.magnetic_field.x = sample.magnetic_field_tesla[0];
    mag.magnetic_field.y = sample.magnetic_field_tesla[1];
    mag.magnetic_field.z = sample.magnetic_field_tesla[2];
    mag_pub_->publish(mag);
  }
}

void TransformerMspBridgeNode::publishAttitude(const AttitudeAngles &angles)
{
  if (attitude_pub_)
  {
    geometry_msgs::msg::Vector3 msg;
    msg.x = angles.roll_deg;
    msg.y = angles.pitch_deg;
    msg.z = angles.yaw_deg;
    attitude_pub_->publish(msg);
  }

  if (broadcast_attitude_tf_)
  {
    broadcastAttitudeTransform(angles);
  }
}

void TransformerMspBridgeNode::publishAltitude(const AltitudeSample &sample)
{
  if (altitude_pub_)
  {
    std_msgs::msg::Float32 msg;
    msg.data = sample.altitude_m;
    altitude_pub_->publish(msg);
  }

  if (vertical_speed_pub_)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_altitude_;
    msg.twist.linear.z = sample.vertical_speed_mps;
    vertical_speed_pub_->publish(msg);
  }

  if (baro_altitude_pub_)
  {
    std_msgs::msg::Float32 msg;
    msg.data = sample.baro_altitude_m;
    baro_altitude_pub_->publish(msg);
  }
}

void TransformerMspBridgeNode::publishGpsRaw(const GpsRawData &data)
{
  if (!gps_fix_pub_ && !gps_vel_pub_)
  {
    return;
  }

  if (gps_fix_pub_)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = now();
    fix.header.frame_id = frame_id_gps_;
    fix.latitude = data.latitude_deg;
    fix.longitude = data.longitude_deg;
    fix.altitude = data.altitude_m;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.status.status = data.fix_type > 0 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                          : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    gps_fix_pub_->publish(fix);
  }

  if (gps_vel_pub_)
  {
    geometry_msgs::msg::TwistStamped vel;
    vel.header.stamp = now();
    vel.header.frame_id = frame_id_gps_;
    vel.twist.linear.x = data.speed_mps;
    vel.twist.angular.z = data.course_rad;
    gps_vel_pub_->publish(vel);
  }
}

void TransformerMspBridgeNode::publishGpsHome(const GpsHomeVector &data)
{
  if (gps_home_vec_pub_)
  {
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.stamp = now();
    vec.header.frame_id = frame_id_gps_;
    vec.vector.x = data.distance_m;
    vec.vector.y = data.direction_deg;
    vec.vector.z = 0.0;
    gps_home_vec_pub_->publish(vec);
  }

  if (gps_home_dist_pub_)
  {
    std_msgs::msg::Float32 dist;
    dist.data = static_cast<float>(data.distance_m);
    gps_home_dist_pub_->publish(dist);
  }
}

void TransformerMspBridgeNode::publishRc(const RcChannelsData &data)
{
  if (!rc_pub_)
  {
    return;
  }
  std_msgs::msg::UInt16MultiArray msg;
  msg.data = data.channels;
  rc_pub_->publish(msg);
}

void TransformerMspBridgeNode::publishServo(const ServoPositionData &data)
{
  if (!servo_pub_)
  {
    return;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.position = data.positions;
  servo_pub_->publish(msg);
}

void TransformerMspBridgeNode::publishMotor(const MotorOutputData &data)
{
  if (!motor_pub_)
  {
    return;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.velocity = data.values;
  motor_pub_->publish(msg);
}

void TransformerMspBridgeNode::publishBatteryAnalog(const BatteryAnalogData &data)
{
  if (!battery_pub_)
  {
    return;
  }
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_imu_;
  msg.present = data.present;
  msg.voltage = data.voltage_v;
  msg.current = data.current_a;
  msg.power_supply_status = data.battery_state;
  msg.percentage = (data.percentage >= 0.0F && data.percentage <= 100.0F) ? data.percentage / 100.0F
                                                                           : std::numeric_limits<float>::quiet_NaN();
  msg.charge = std::numeric_limits<float>::quiet_NaN();
  msg.capacity = std::numeric_limits<float>::quiet_NaN();
  msg.design_capacity = std::numeric_limits<float>::quiet_NaN();

  if (data.capacity_unit == 0 && std::isfinite(data.remaining_capacity_mah))
  {
    msg.charge = data.remaining_capacity_mah / 1000.0F;
  }
  else if (data.capacity_unit == 1 && std::isfinite(data.remaining_capacity_mwh) && data.voltage_v > 0.0F)
  {
    msg.charge = data.remaining_capacity_mwh / (1000.0F * data.voltage_v);
  }

  {
    std::lock_guard<std::mutex> lock(battery_config_mutex_);
    if (last_battery_config_)
    {
      const auto &config = *last_battery_config_;
      if (config.capacity_unit == 0 && std::isfinite(config.capacity_mah))
      {
        msg.capacity = config.capacity_mah / 1000.0F; // convert mAh to Ah
        msg.design_capacity = msg.capacity;
      }
      else if (config.capacity_unit == 1 && std::isfinite(config.capacity_mwh) && data.voltage_v > 0.0F)
      {
        const float capacity_ah = config.capacity_mwh / (1000.0F * data.voltage_v);
        msg.capacity = capacity_ah;
        msg.design_capacity = capacity_ah;
      }
      msg.cell_voltage.resize(config.cell_count);
      std::fill(msg.cell_voltage.begin(), msg.cell_voltage.end(), std::numeric_limits<float>::quiet_NaN());
    }
    else
    {
      msg.cell_voltage.resize(data.cell_count);
      std::fill(msg.cell_voltage.begin(), msg.cell_voltage.end(), std::numeric_limits<float>::quiet_NaN());
    }
  }
  battery_pub_->publish(msg);
}

void TransformerMspBridgeNode::handleBatteryConfig(const BatteryConfigData &data)
{
  std::lock_guard<std::mutex> lock(battery_config_mutex_);
  last_battery_config_ = data;
}

void TransformerMspBridgeNode::publishInavStatus(const InavStatusData &data)
{
  if (!inav_status_pub_)
  {
    return;
  }
  transformer_msp_bridge::msg::MspInavStatus msg;
  msg.cycle_time_us = data.cycle_time_us;
  msg.i2c_errors = data.i2c_errors;
  msg.sensor_status = data.sensor_status_mask;
  msg.cpu_load_percent = data.cpu_load_percent;
  msg.config_profile = data.config_profile;
  msg.battery_profile = data.battery_profile;
  msg.mixer_profile = data.mixer_profile;
  msg.arming_flags = data.arming_flags;
  msg.active_modes = data.active_modes;
  inav_status_pub_->publish(msg);
}

void TransformerMspBridgeNode::publishInavGeneric(const InavGenericFrame &frame)
{
  if (!inav_generic_pub_)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = frame.description;
  status.hardware_id = "fc";
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "payload";
  kv.value = detail::to_hex(frame.payload, false);
  status.values.push_back(std::move(kv));
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  inav_generic_pub_->publish(array);
}

void TransformerMspBridgeNode::publishStatusEx(const SystemStatusExData &data)
{
  if (!status_pub_)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "status_ex";
  status.hardware_id = "fc";
  detail::append_key_value(status, "cycle_time_us", data.cycle_time_us);
  detail::append_key_value(status, "i2c_errors", data.i2c_errors);
  detail::append_key_value(status, "sensors_mask", data.sensors_mask);
  detail::append_key_value(status, "mode_flags", data.mode_flags);
  detail::append_key_value(status, "profile", data.profile);
  detail::append_key_value(status, "system_load_pct", data.system_load_pct);
  detail::append_key_value(status, "gyro_cycle_time_us", data.gyro_cycle_time_us);
  detail::append_key_value(status, "last_arm_disable", data.last_arm_disable);
  detail::append_key_value(status, "arming_flags", data.arming_flags);
  detail::append_key_value(status, "acc_cal_running", data.acc_cal_running);
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  status_pub_->publish(array);
}

void TransformerMspBridgeNode::publishSensorStatus(const SystemSensorStatusData &data)
{
  if (!sensor_status_pub_)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "sensor_status";
  status.hardware_id = "fc";
  if (data.has_masks)
  {
    detail::append_key_value(status, "present_mask", data.present_mask);
    detail::append_key_value(status, "failing_mask", data.failing_mask);
  }
  else if (!data.raw_payload.empty())
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "raw";
    kv.value = detail::to_hex(data.raw_payload);
    status.values.push_back(std::move(kv));
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  sensor_status_pub_->publish(array);
}

void TransformerMspBridgeNode::publishSensorConfig(const SystemSensorConfigData &data)
{
  if (!sensor_status_pub_)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "sensor_config";
  status.hardware_id = "fc";
  for (std::size_t i = 0; i < data.hardware_ids.size(); ++i)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "hardware[" + std::to_string(i) + "]";
    kv.value = std::to_string(data.hardware_ids[i]);
    status.values.push_back(std::move(kv));
  }
  if (!data.raw_tail.empty())
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "raw_tail";
    kv.value = detail::to_hex(data.raw_tail);
    status.values.push_back(std::move(kv));
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  sensor_status_pub_->publish(array);
}

void TransformerMspBridgeNode::publishGpsStatistics(const SystemGpsStatsData &data)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "gps_statistics";
  status.hardware_id = "fc";
  detail::append_key_value(status, "errors", data.errors);
  detail::append_key_value(status, "timeouts", data.timeouts);
  detail::append_key_value(status, "packet_count", data.packet_count);
  detail::append_key_value(status, "packet_rejected", data.packet_rejected);
  detail::append_key_value(status, "packet_ignored", data.packet_ignored);
  detail::append_key_value(status, "packet_crc_error", data.packet_crc_error);
  detail::append_key_value(status, "gps_reset_flags", data.gps_reset_flags);

  if (gps_stats_status_pub_)
  {
    gps_stats_status_pub_->publish(status);
  }

  if (gps_stats_array_pub_)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    gps_stats_array_pub_->publish(array);
  }
}

void TransformerMspBridgeNode::publishRcTuning(const RcTuningData &data)
{
  if (!rc_tuning_pub_)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "rc_tuning";
  status.hardware_id = "fc";
  detail::append_key_value(status, "rc_rate", data.rc_rate);
  detail::append_key_value(status, "rc_expo", data.rc_expo);
  detail::append_key_value(status, "thr_mid", data.throttle_mid);
  detail::append_key_value(status, "thr_expo", data.throttle_expo);
  detail::append_key_value(status, "dyn_throttle_pid", data.dyn_throttle_pid);
  detail::append_key_value(status, "rc_yaw_expo", data.rc_yaw_expo);
  detail::append_key_value(status, "rc_yaw_rate", data.rc_yaw_rate);
  detail::append_key_value(status, "rc_pitch_rate", data.rc_pitch_rate);
  detail::append_key_value(status, "rc_roll_rate", data.rc_roll_rate);
  detail::append_key_value(status, "tpa_breakpoint", data.tpa_breakpoint);
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  rc_tuning_pub_->publish(array);
}

void TransformerMspBridgeNode::publishRtc(const SystemRtcData &data)
{
  if (!rtc_pub_)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "rtc";
  status.hardware_id = "fc";
  detail::append_key_value(status, "seconds", data.seconds);
  detail::append_key_value(status, "millis", data.millis);
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  rtc_pub_->publish(array);
}

void TransformerMspBridgeNode::broadcastAttitudeTransform(const AttitudeAngles &angles)
{
  if (!tf_broadcaster_ || attitude_tf_parent_frame_.empty() || attitude_tf_child_frame_.empty())
  {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id = attitude_tf_parent_frame_;
  tf_msg.child_frame_id = attitude_tf_child_frame_;
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;

  static constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double roll = static_cast<double>(angles.roll_deg) * kDegToRad;
  const double pitch = static_cast<double>(angles.pitch_deg) * kDegToRad;
  const double yaw = static_cast<double>(angles.yaw_deg) * kDegToRad;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf_msg.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(tf_msg);
}

} // namespace transformer_msp_bridge
