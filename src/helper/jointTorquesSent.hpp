
#ifndef ZAKERIMANESH_JOINT_TORQUES_SENT_HPP
#define ZAKERIMANESH_JOINT_TORQUES_SENT_HPP

#include <Eigen/Dense>

namespace zakerimanesh {
inline std::array<double, 7> jointTorquesSent(
    const Eigen::Matrix<double, 7, 1>& joint_torques) noexcept {
  std::array<double, 7> joints_torque_sent;
  // since franka::Torques constructor need std::array<double, 7>

  Eigen::Map<Eigen::Matrix<double, 7, 1>> jointsTorques_2_arry_interface(joints_torque_sent.data());
  jointsTorques_2_arry_interface = joint_torques;
  return joints_torque_sent;
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_JOINT_TORQUES_SENT_HPP