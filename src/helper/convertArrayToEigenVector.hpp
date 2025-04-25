#ifndef ZAKERIMANESH_FRANKA_LOCAL_CONVERT_ARRAY_TO_EIGEN_VECTOR_HPP
#define ZAKERIMANESH_FRANKA_LOCAL_CONVERT_ARRAY_TO_EIGEN_VECTOR_HPP

// #pragma once

#include <Eigen/Dense>

namespace zakerimanesh {
template <std::size_t N>
inline Eigen::Matrix<double, N, 1> convertArrayToEigenVector(
    const std::array<double, N>& array_input) noexcept {
  return Eigen::Map<const Eigen::Matrix<double, N, 1>>(array_input.data());
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_FRANKA_LOCAL_CONVERT_ARRAY_TO_EIGEN_VECTOR_HPP