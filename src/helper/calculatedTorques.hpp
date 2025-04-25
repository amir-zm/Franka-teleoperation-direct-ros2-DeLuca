#ifndef ZAKERIMANESH_FRANKA_LOCAL_CALCULATED_TORQUES_HPP
#define ZAKERIMANESH_FRANKA_LOCAL_CALCULATED_TORQUES_HPP

#include <franka/model.h>
#include <Eigen/Dense>

namespace zakerimanesh {
inline Eigen::Matrix<double, 7, 1> calculatedTorques(
    const Eigen::Matrix<double, 6, 6>& stiffness,
    const Eigen::Matrix<double, 6, 6>& damping,
    const Eigen::Matrix<double, 6, 1>& end_effector_full_pose_error,
    const Eigen::Matrix<double, 6, 1>& ee_velocity,
    const Eigen::Matrix<double, 7, 6>& jacobian_transpose,
    const Eigen::Matrix<double, 7, 1>& robot_coriolis_times_dq) noexcept {

  // online joint torques (7 joints)
  return jacobian_transpose * (-damping * ee_velocity - stiffness * end_effector_full_pose_error) +
         robot_coriolis_times_dq;
}

}  // namespace zakerimanesh
#endif  // ZAKERIMANESH_FRANKA_LOCAL_CALCULATED_TORQUES_HPP
