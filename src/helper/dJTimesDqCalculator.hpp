
#ifndef ZAKERIMANESH_DJ_TIMES_DQ_CALCULATOR_HPP
#define ZAKERIMANESH_DJ_TIMES_DQ_CALCULATOR_HPP

#include <franka/model.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>

#include "convertArrayToEigenVector.hpp"

namespace zakerimanesh {
inline Eigen::Matrix<double, 6, 1> dJTimesDqCalculator(
    const franka::RobotState& robotOnlineState,
    const franka::Model& model,
    Eigen::Matrix<double, 6, 7>& dot_jacobian_matrix,
    std::array<double, 42>& previous_jacobian) noexcept {
      
  std::array<double, 42> J_curr = model.zeroJacobian(franka::Frame::kEndEffector, robotOnlineState);

  double inv_dt = 1000;

  // 3) finite‐difference to get Jdot
  std::array<double, 42> Jdot;
  for (int i = 0; i < 42; ++i) {
    Jdot[i] = (J_curr[i] - previous_jacobian[i]) * inv_dt;
  }

  // 4) (optional) multiply by joint velocities to get Cartesian “Coriolis” term
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> dot_jacobian_matrix_temp(Jdot.data());
  dot_jacobian_matrix = dot_jacobian_matrix_temp;

  Eigen::Matrix<double, 6, 1> dJ_times_dq;
  dJ_times_dq = dot_jacobian_matrix * convertArrayToEigenVector(robotOnlineState.dq);

  // store for next iteration
  previous_jacobian = J_curr;
  return dJ_times_dq;
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_DJ_TIMES_DQ_CALCULATOR_HPP