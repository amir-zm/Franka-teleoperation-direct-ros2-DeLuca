#ifndef ZAKERIMANESH_FRANKA_LOCAL_CONVERT_ARRAY_TO_EIGEN_MATRIX_HPP
#define ZAKERIMANESH_FRANKA_LOCAL_CONVERT_ARRAY_TO_EIGEN_MATRIX_HPP

// #pragma once

#include <Eigen/Dense>
#include <array>

namespace zakerimanesh {
template <std::size_t N, std::size_t M>
inline Eigen::Matrix<double, N, M> convertArrayToEigenMatrix(
    const std::array<double, N * M>& array_input) noexcept {
  // since franka::Torques constructor need std::array<double, 7>
  return Eigen::Map<const Eigen::Matrix<double, N, M>>(array_input.data());
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_FRANKA_LOCAL_CONVERT_ARRAY_TO_EIGEN_MATRIX_HPP