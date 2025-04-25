#ifndef ZAKERIMANESH_CORIOLIS_TIME_DQ_HPP
#define ZAKERIMANESH_CORIOLIS_TIME_DQ_HPP

#include <franka/model.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>

#include "convertArrayToEigenVector.hpp"

// Computing on-line end-effector linear velocity (xd = Jacobian * djoint_velocities)
// model.zeroJacobian(franka::Frame::kEndEffector, robotOnlineState) gives std::array<double,42>
// model.zeroJacobian(franka::Frame::kEndEffector, robotOnlineState).data() return a pointer to the
// first element! so under the hood is like double matrix[6][7] and using returned pointer it fills
// the matrix!

namespace zakerimanesh {
inline Eigen::Matrix<double, 7, 1> coriolisTimesDqVector(const franka::RobotState& robotOnlineState,
                                                         const franka::Model& model) noexcept {
  return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(model.coriolis(robotOnlineState).data());}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_CORIOLIS_TIME_DQ_HPP