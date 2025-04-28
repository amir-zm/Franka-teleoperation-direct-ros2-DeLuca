#ifndef ZAKERIMANESH_JOINTSPACE_CALCULATED_TORQUES_HPP
#define ZAKERIMANESH_JOINTSPACE_CALCULATED_TORQUES_HPP

#include <Eigen/Dense>

namespace zakerimanesh {
inline Eigen::Matrix<double, 7, 1> remoteCalculatedTorques(
    const Eigen::Matrix<double, 7, 7>& stiffness,
    const Eigen::Matrix<double, 7, 7>& damping,
    const Eigen::Matrix<double, 7, 1>& joints_positions_error,
    const Eigen::Matrix<double, 7, 1>& joints_velocities_error,
    const Eigen::Matrix<double, 7, 1>& robot_coriolis_times_dq) noexcept {
    
  // online joint torques (7 joints)
  return -damping * joints_velocities_error - stiffness * joints_positions_error +
         robot_coriolis_times_dq;
}

}  // namespace zakerimanesh
#endif  // ZAKERIMANESH_JOINTSPACE_CALCULATED_TORQUES_HPP