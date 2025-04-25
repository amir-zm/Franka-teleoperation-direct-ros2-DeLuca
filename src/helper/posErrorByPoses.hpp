#ifndef ZAKERIMANESH_POS_ERROR_BY_POSES_HPP
#define ZAKERIMANESH_POS_ERROR_BY_POSES_HPP

#include <Eigen/Dense>

namespace zakerimanesh {
inline Eigen::Matrix<double, 3, 1> posErrorByPoses(const Eigen::Affine3d& pose_1,
                                                   const Eigen::Affine3d& pose_2) noexcept {
  return pose_1.translation() - pose_2.translation();
}
}  // namespace zakerimanesh
#endif  // ZAKERIMANESH_POS_ERROR_BY_POSES_HPP