#ifndef ZAKERIMANESH_ROTATION_ERROR_HPP
#define ZAKERIMANESH_ROTATION_ERROR_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace zakerimanesh {
inline Eigen::Matrix<double, 3, 1> rotationErUnifiedAngleAxis(
    const Eigen::Matrix<double, 3, 3>& orientatation_error) noexcept {
  // extracting needed rotation_axis and rotation_angle based on end_effector_orientation_error
  Eigen::AngleAxisd extract_axis_angle_EE(orientatation_error);

  // unify rotation axis and rotation angle to one unified vector
  return extract_axis_angle_EE.axis() * extract_axis_angle_EE.angle();
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_ROTATION_ERROR_HPP