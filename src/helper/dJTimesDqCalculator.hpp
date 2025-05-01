
#ifndef ZAKERIMANESH_DJ_TIMES_DQ_CALCULATOR_HPP
#define ZAKERIMANESH_DJ_TIMES_DQ_CALCULATOR_HPP

#include <franka/model.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>

#include "convertArrayToEigenVector.hpp"

namespace zakerimanesh {
inline Eigen::Matrix<double, 6, 1> dJTimesDqCalculator(
    Eigen::Matrix<double, 6, 7>& previous_jacobian_matrix,
    Eigen::Matrix<double, 6, 7>& current_jacobian_matrix,
    Eigen::Matrix<double, 6, 7>& dot_jacobian_matrix,
    Eigen::Matrix<double, 7, 1>& joint_velocities) noexcept {
  double inv_dt = 1000;
  dot_jacobian_matrix = inv_dt * (current_jacobian_matrix - previous_jacobian_matrix);

  // store for next iteration
  previous_jacobian_matrix = current_jacobian_matrix;
  return dot_jacobian_matrix * joint_velocities;
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_DJ_TIMES_DQ_CALCULATOR_HPP