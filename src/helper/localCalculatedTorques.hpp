#ifndef ZAKERIMANESH_FRANKA_LOCAL_CALCULATED_TORQUES_HPP
#define ZAKERIMANESH_FRANKA_LOCAL_CALCULATED_TORQUES_HPP

#include <franka/model.h>
#include <Eigen/Dense>

namespace zakerimanesh {
inline Eigen::Matrix<double, 7, 1> localCalculatedTorques(
    const Eigen::Matrix<double, 6, 6>& inertia_inverse_matrix_,
    const Eigen::Matrix<double, 6, 6>& stiffness_matrix_,
    const Eigen::Matrix<double, 6, 6>& damping_matrix_,
    const Eigen::Matrix<double, 6, 1>& end_effector_full_pose_error,
    const Eigen::Matrix<double, 6, 1>& ee_velocity,
    const Eigen::Matrix<double, 7, 6>& jacobian_matrix_transpose,
    const Eigen::Matrix<double, 7, 6>& psuedo_jacobian_matrix,
    const Eigen::Matrix<double, 7, 1>& robot_coriolis_times_dq,
    Eigen::Matrix<double, 7, 7> robot_inertia_matrix,
    Eigen::Matrix<double, 6, 1>& dJ_times_dq) noexcept {
  // online joint torques (7 joints)
  // return jacobian_matrix_transpose * ( -damping_matrix_ * ee_velocity - stiffness_matrix_ *
  // end_effector_full_pose_error) +
  //        robot_coriolis_times_dq;

  // online joint torques (7 joints)
  return robot_inertia_matrix * psuedo_jacobian_matrix *
             ( - dJ_times_dq - inertia_inverse_matrix_ * damping_matrix_ * ee_velocity -
              inertia_inverse_matrix_ * stiffness_matrix_ * end_effector_full_pose_error) +
         robot_coriolis_times_dq;
}

}  // namespace zakerimanesh
#endif  // ZAKERIMANESH_FRANKA_LOCAL_CALCULATED_TORQUES_HPP
