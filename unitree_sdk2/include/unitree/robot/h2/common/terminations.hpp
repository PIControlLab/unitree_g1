#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <eigen3/Eigen/Dense>

#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace unitree {
namespace robot {
namespace h2 {

inline bool bad_orientation(const unitree_hg::msg::dds_::LowState_& lowstate,
                            float limit_angle = 1.0f) {
  const auto& imu = lowstate.imu_state();
  Eigen::Quaternionf quat(imu.quaternion()[0], imu.quaternion()[1],
                          imu.quaternion()[2], imu.quaternion()[3]);
  const Eigen::Vector3f projected_gravity_b =
      quat.conjugate() * Eigen::Vector3f(0, 0, -1);
  return std::fabs(std::acos(-projected_gravity_b[2])) > limit_angle;
}

inline bool joint_vel_out_of_limit(
    const unitree_hg::msg::dds_::LowState_& lowstate,
    float limit_vel = 10.0f) {
  const auto& motors = lowstate.motor_state();
  return std::any_of(motors.begin(), motors.end(), [limit_vel](const auto& motor) {
    return std::fabs(motor.dq()) > limit_vel;
  });
}

inline bool ang_vel_out_of_limit(
    const unitree_hg::msg::dds_::LowState_& lowstate,
    float limit_vel = 6.0f) {
  const auto& gyroscope = lowstate.imu_state().gyroscope();
  return std::any_of(gyroscope.begin(), gyroscope.end(), [limit_vel](float value) {
    return std::fabs(value) > limit_vel;
  });
}

inline bool motor_winding_overheat(
    const unitree_hg::msg::dds_::LowState_& lowstate,
    float limit_temp = 120.0f) {
  const auto& motors = lowstate.motor_state();
  return std::any_of(motors.begin(), motors.end(), [limit_temp](const auto& motor) {
    return motor.temperature()[1] > limit_temp;
  });
}

inline bool motor_casing_overheat(
    const unitree_hg::msg::dds_::LowState_& lowstate,
    float limit_temp = 85.0f) {
  const auto& motors = lowstate.motor_state();
  return std::any_of(motors.begin(), motors.end(), [limit_temp](const auto& motor) {
    return motor.temperature()[0] > limit_temp;
  });
}

inline bool low_battery(const unitree_hg::msg::dds_::BmsState_& bms_state,
                        float limit_soc = 20.0f) {
  return bms_state.soc() < limit_soc;
}

inline bool lost_connection(
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>& subscriber,
    int64_t timeout_ms = 1000) {
  const auto now = unitree::common::GetCurrentMonotonicTimeNanosecond();
  const auto elapsed_ms =
      (now - subscriber->GetLastDataAvailableTime()) / 1e6;
  return elapsed_ms > timeout_ms;
}

}  // namespace h2
}  // namespace robot
}  // namespace unitree
