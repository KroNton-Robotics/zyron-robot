#include <cassert>

#include "control_logic.h"

using zyron_firmware::ImuHealthGate;
using zyron_firmware::MotorRampState;

int main()
{
  MotorRampState forward;
  zyron_firmware::resetMotorRamp(forward, 0);
  assert(zyron_firmware::updateMotorRamp(forward, 255, 20, 5, 16, 20) == 64);

  MotorRampState reverse;
  zyron_firmware::resetMotorRamp(reverse, 0);
  assert(zyron_firmware::updateMotorRamp(reverse, -255, 20, 5, 16, 20) == -64);

  MotorRampState direction_change;
  zyron_firmware::resetMotorRamp(direction_change, 0);
  assert(zyron_firmware::updateMotorRamp(direction_change, 255, 5, 5, 16, 20) == 16);
  assert(zyron_firmware::updateMotorRamp(direction_change, -255, 10, 5, 16, 20) == 0);
  assert(zyron_firmware::updateMotorRamp(direction_change, -255, 25, 5, 16, 20) == 0);
  assert(zyron_firmware::updateMotorRamp(direction_change, -255, 30, 5, 16, 20) == 0);
  assert(zyron_firmware::updateMotorRamp(direction_change, -255, 35, 5, 16, 20) == -16);

  ImuHealthGate imu;
  zyron_firmware::resetImuHealthGate(imu, 0);
  zyron_firmware::noteImuPacket(imu, 10, 3);
  zyron_firmware::noteImuPacket(imu, 20, 3);
  assert(!imu.motion_allowed);
  zyron_firmware::noteImuPacket(imu, 30, 3);
  assert(imu.motion_allowed);
  assert(!zyron_firmware::imuPacketIsStale(imu, 129, 100));
  assert(zyron_firmware::imuPacketIsStale(imu, 130, 100));
  zyron_firmware::resetImuHealthGate(imu, 130);
  assert(!imu.motion_allowed);

  zyron_firmware::WheelControlConfig control_config;
  control_config.max_acceleration_rad_s2 = 10.0F;
  control_config.start_threshold_rad_s = 0.5F;
  control_config.stop_threshold_rad_s = 0.2F;
  control_config.pwm_min = 60;
  control_config.pwm_max = 255;
  control_config.kp = 1.0F;
  control_config.ki = 2.0F;

  zyron_firmware::WheelControlState positive;
  zyron_firmware::WheelControlState negative;
  const int positive_pwm = zyron_firmware::updateWheelControl(
    positive, 5.0F, 0.0F, 0.01F, 10, control_config, true);
  const int negative_pwm = zyron_firmware::updateWheelControl(
    negative, -5.0F, 0.0F, 0.01F, 10, control_config, true);
  assert(positive_pwm == -negative_pwm);
  assert(positive.ramped_target_rad_s == -negative.ramped_target_rad_s);

  zyron_firmware::WheelControlState stopped;
  assert(zyron_firmware::updateWheelControl(
    stopped, 0.1F, 0.0F, 0.01F, 10, control_config, true) == 0);
  assert(zyron_firmware::updateWheelControl(
    stopped, 5.0F, 0.0F, 0.01F, 20, control_config, false) == 0);
  assert(stopped.integral == 0.0F);

  zyron_firmware::WheelControlState saturated;
  saturated.integral = 250.0F;
  const int saturated_pwm = zyron_firmware::updateWheelControl(
    saturated, 20.0F, 0.0F, 0.1F, 100, control_config, true);
  assert(saturated_pwm == 255);
  assert(saturated.saturated);
  assert(saturated.integral == 250.0F);

  zyron_firmware::WheelControlState reversal;
  reversal.ramped_target_rad_s = 0.1F;
  reversal.demand_active = true;
  assert(zyron_firmware::updateWheelControl(
    reversal, -5.0F, 0.0F, 0.01F, 10, control_config, true) == 0);
  assert(reversal.direction_hold);
  assert(zyron_firmware::updateWheelControl(
    reversal, -5.0F, 0.0F, 0.01F, 29, control_config, true) == 0);
  assert(reversal.ramped_target_rad_s == 0.0F);
  zyron_firmware::updateWheelControl(
    reversal, -5.0F, 0.0F, 0.01F, 30, control_config, true);
  assert(reversal.ramped_target_rad_s < -0.099F &&
    reversal.ramped_target_rad_s > -0.101F);

  const float count_velocity = zyron_firmware::estimateEncoderVelocity(
    3, 10000, 1000, 1, 228.0F, 0.01F, 150000);
  assert(count_velocity > 8.2F && count_velocity < 8.3F);
  const float period_velocity = zyron_firmware::estimateEncoderVelocity(
    0, 20000, 10000, -1, 228.0F, 0.01F, 150000);
  assert(period_velocity < -1.3F && period_velocity > -1.4F);
  assert(zyron_firmware::estimateEncoderVelocity(
    0, 20000, 150001, 1, 228.0F, 0.01F, 150000) == 0.0F);
  assert(!zyron_firmware::commandIsStale(199, 0, 200));
  assert(zyron_firmware::commandIsStale(200, 0, 200));

  return 0;
}
