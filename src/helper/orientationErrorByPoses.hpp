#ifndef ZAKERIMANESH_ORIENTATION_ERROR_BY_POSES_HPP
#define ZAKERIMANESH_ORIENTATION_ERROR_BY_POSES_HPP

#include <Eigen/Dense>

namespace zakerimanesh {
inline Eigen::Matrix<double, 3, 3> orientationErrorByPoses(const Eigen::Affine3d& pose_1,
                                                           const Eigen::Affine3d& pose_2) noexcept {
  return pose_1.linear().transpose() * pose_2.linear();
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_ORIENTATION_ERROR_BY_POSES_HPP
