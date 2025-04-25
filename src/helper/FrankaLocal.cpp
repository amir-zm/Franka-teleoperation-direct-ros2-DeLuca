
// ROS C++ wrapper
#include <rclcpp/rclcpp.hpp>

// from libfranka low-level API
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include "/home/mobilerobot/franka_ws/src/libfranka/examples/examples_common.h"

#include <pthread.h>
#include <sched.h>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "FrankaLocal.hpp"
#include "calculatedTorques.hpp"
#include "convertArrayToEigenMatrix.hpp"
#include "convertArrayToEigenVector.hpp"
#include "coriolisTimesDqVector.hpp"
#include "jacobianMatrix.hpp"
#include "jointTorquesSent.hpp"
#include "orientationErrorByPoses.hpp"
#include "posErrorByPoses.hpp"
#include "rotationErUnifiedAngleAxis.hpp"

namespace zakerimanesh {
FrankaLocal::FrankaLocal() : Node("franka_teleoperation_local_node"), stop_control_loop_{false} {
  this->declare_parameter<std::vector<double>>("stiffness", {tr_s, tr_s, tr_s, r_s, r_s, r_s});
  this->declare_parameter<std::vector<double>>("damping", {tr_d, tr_d, tr_d, r_d, r_d, r_d});

  // Retrieve parameter values. They can be overridden by YAML files or launch files.
  std::vector<double> stiffness_values = this->get_parameter("stiffness").as_double_array();
  std::vector<double> damping_values = this->get_parameter("damping").as_double_array();

  // Fill in the diagonal elements
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> stifness_values_mapped(stiffness_values.data());
  stiffness_.diagonal() = stifness_values_mapped;

  Eigen::Map<const Eigen::Matrix<double, 6, 1>> damping_values_mapped(damping_values.data());
  damping_.diagonal() = damping_values_mapped;

  this->declare_parameter<std::string>("robot_ip", "192.168.1.11");
  robot_ip_ = this->get_parameter("robot_ip").as_string();

  control_thread_ = std::thread(&FrankaLocal::controlLoop, this);
}

FrankaLocal::~FrankaLocal() {
  stop_control_loop_ = true;
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

void FrankaLocal::controlLoop() {
  // Pin this thread to core 2
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(2, &cpuset);
  pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

  RCLCPP_INFO(this->get_logger(), "Connecting to franka local robot ...");

  try {
    franka::Robot robot(robot_ip_);
    setDefaultBehavior(robot);

    franka::Model model = robot.loadModel();

    double torque_thresholds = 100;
    double force_thresholds = 100;

    // set collision behavior
    const std::array<double, 7> lower_torque_thresholds_acceleration = {
        torque_thresholds, torque_thresholds, torque_thresholds, torque_thresholds,
        torque_thresholds, torque_thresholds, torque_thresholds};
    const std::array<double, 7> upper_torque_thresholds_acceleration = {
        torque_thresholds, torque_thresholds, torque_thresholds, torque_thresholds,
        torque_thresholds, torque_thresholds, torque_thresholds};
    const std::array<double, 6> lower_force_thresholds_acceleration = {
        force_thresholds, force_thresholds, force_thresholds,
        force_thresholds, force_thresholds, force_thresholds};
    const std::array<double, 6> upper_force_thresholds_acceleration = {
        force_thresholds, force_thresholds, force_thresholds,
        force_thresholds, force_thresholds, force_thresholds};

    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration);

    franka::RobotState robot_initial_state = robot.readOnce();

    // desired pose = initial pose (position+orientation)
    Eigen::Affine3d end_effector_desired_pose;
    end_effector_desired_pose.matrix() =
        convertArrayToEigenMatrix<4, 4>(robot_initial_state.O_T_EE);

    Eigen::Affine3d end_effector_online_pose;
    Eigen::Matrix<double, 6, 1> end_effector_full_pose_error;
    Eigen::Matrix<double, 7, 1> robot_coriolis_times_dq;
    Eigen::Matrix<double, 6, 7> jacobian_matrix;
    Eigen::Matrix<double, 7, 6> jacobian_matrix_transpose;
    Eigen::Matrix<double, 6, 1> ee_velocity;
    Eigen::Matrix<double, 7, 1> tau_output;

    // wrapper
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback;
    // impedance_control_call
    RCLCPP_INFO(this->get_logger(), "local control loop ...");

    impedance_control_callback = [&](const franka::RobotState& robotOnlineState,
                                     franka::Duration /*duration*/) -> franka::Torques {
      if (stop_control_loop_) {
        // the special MotionFinished return halts the control loop immediately
        return franka::MotionFinished(franka::Torques{{0, 0, 0, 0, 0, 0, 0}});
      }

      // online end-effector pose (position+orientation)
      end_effector_online_pose.matrix() = convertArrayToEigenMatrix<4, 4>(robotOnlineState.O_T_EE);
      Eigen::Matrix<double, 3, 3> orientation_error_in_base_frame =
          end_effector_online_pose.rotation();

      // full end-ffector pose error (position+orientation) "the current pose away from the
      // desired"
      end_effector_full_pose_error
          << posErrorByPoses(end_effector_online_pose, end_effector_desired_pose),
          -orientation_error_in_base_frame *
              rotationErUnifiedAngleAxis(
                  orientationErrorByPoses(end_effector_online_pose, end_effector_desired_pose));

      // robot coriolis (C(q,dq)*dq)
      robot_coriolis_times_dq = coriolisTimesDqVector(robotOnlineState, model);

      // robot jacobian (J is 6*7)
      jacobian_matrix = jacobianMatrix(robotOnlineState, model);
      const Eigen::Matrix<double, 6, 7>& jacobian_matrix_alias = jacobian_matrix;

      // jacobian_transpose
      // instead of using jacobian_matrix.transpose() now JM.transpose(), to avoid copying
      jacobian_matrix_transpose = jacobian_matrix_alias.transpose();
      const Eigen::Matrix<double, 7, 6>& jacobian_matrix_transpose_alias =
          jacobian_matrix_transpose;
      ;

      // Computing on-line end-effector linear velocity (xd = Jacobian * djoint_velocities)
      ee_velocity = jacobian_matrix_alias * convertArrayToEigenVector<7>(robotOnlineState.dq);

      // calculated torque
      tau_output =
          calculatedTorques(stiffness_, damping_, end_effector_full_pose_error, ee_velocity,
                            jacobian_matrix_transpose_alias, robot_coriolis_times_dq);

      return jointTorquesSent(tau_output);
    };
    // ROS action 1KHZ
    RCLCPP_INFO(this->get_logger(), "Starting local impedance control loop...");
    robot.control(impedance_control_callback);
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Franka Exception: %s", e.what());
  }
}
}  // namespace zakerimanesh
