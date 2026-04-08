// 该文件单独定义了节点的decoder的configuration
#include "transformer_msp_bridge/transformer_msp_bridge_node.hpp"

#include "transformer_msp_bridge/decoders/altitude_decoder.hpp"
#include "transformer_msp_bridge/decoders/attitude_decoder.hpp"
#include "transformer_msp_bridge/decoders/battery_decoder.hpp"
#include "transformer_msp_bridge/decoders/gps_decoder.hpp"
#include "transformer_msp_bridge/decoders/imu_decoder.hpp"
#include "transformer_msp_bridge/decoders/inav_generic_decoder.hpp"
#include "transformer_msp_bridge/decoders/inav_status_decoder.hpp"
#include "transformer_msp_bridge/decoders/rc_decoder.hpp"
#include "transformer_msp_bridge/decoders/servo_motor_decoder.hpp"
#include "transformer_msp_bridge/decoders/system_decoder.hpp"

namespace transformer_msp_bridge
{
void TransformerMspBridgeNode::configureDecoders()
{
  decoders_.clear();

  decoders_.emplace_back(std::make_unique<ImuDecoder>(ImuDecoder::Callback([this](const ImuSample &sample) {
    publishImu(sample);
  })));

  decoders_.emplace_back(std::make_unique<AttitudeDecoder>(AttitudeDecoder::Callback([this](const AttitudeAngles &angles) {
    publishAttitude(angles);
  })));

  decoders_.emplace_back(std::make_unique<AltitudeDecoder>(AltitudeDecoder::Callback([this](const AltitudeSample &sample) {
    publishAltitude(sample);
  })));

  GpsDecoder::Callbacks gps_callbacks;
  gps_callbacks.raw = [this](const GpsRawData &data) { publishGpsRaw(data); };
  gps_callbacks.home = [this](const GpsHomeVector &data) { publishGpsHome(data); };
  decoders_.emplace_back(std::make_unique<GpsDecoder>(gps_callbacks));

  decoders_.emplace_back(std::make_unique<RcDecoder>(RcDecoder::Callback([this](const RcChannelsData &data) {
    publishRc(data);
  })));

  ServoMotorDecoder::Callbacks servo_callbacks;
  servo_callbacks.servo = [this](const ServoPositionData &data) { publishServo(data); };
  servo_callbacks.motor = [this](const MotorOutputData &data) { publishMotor(data); };
  decoders_.emplace_back(std::make_unique<ServoMotorDecoder>(servo_callbacks));

  BatteryDecoder::Callbacks battery_callbacks;
  battery_callbacks.analog = [this](const BatteryAnalogData &data) { publishBatteryAnalog(data); };
  battery_callbacks.config = [this](const BatteryConfigData &data) { handleBatteryConfig(data); };
  decoders_.emplace_back(std::make_unique<BatteryDecoder>(battery_callbacks));

  InavStatusDecoder::Callbacks inav_callbacks;
  inav_callbacks.status = [this](const InavStatusData &data) { publishInavStatus(data); };
  if (debug_msp_)
  {
    inav_callbacks.log_info = [this](const std::string &message) { RCLCPP_INFO(get_logger(), "%s", message.c_str()); };
    inav_callbacks.log_first_frame = true;
  }
  decoders_.emplace_back(std::make_unique<InavStatusDecoder>(inav_callbacks));

  decoders_.emplace_back(std::make_unique<InavGenericDecoder>(InavGenericDecoder::FrameCallback([this](const InavGenericFrame &frame) {
    publishInavGeneric(frame);
  })));

  SystemDecoder::Callbacks system_callbacks;
  system_callbacks.status_ex = [this](const SystemStatusExData &data) { publishStatusEx(data); };
  system_callbacks.sensor_status = [this](const SystemSensorStatusData &data) { publishSensorStatus(data); };
  system_callbacks.sensor_config = [this](const SystemSensorConfigData &data) { publishSensorConfig(data); };
  system_callbacks.gps_statistics = [this](const SystemGpsStatsData &data) { publishGpsStatistics(data); };
  system_callbacks.rc_tuning = [this](const RcTuningData &data) { publishRcTuning(data); };
  system_callbacks.rtc = [this](const SystemRtcData &data) { publishRtc(data); };
  decoders_.emplace_back(std::make_unique<SystemDecoder>(system_callbacks));
}

} // namespace transformer_msp_bridge
