
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

#include "CustomImpedanceNode.hpp"
#include "calculatedTorques.hpp"
#include "convertArrayToEigenMatrix.hpp"
#include "convertArrayToEigenVector.hpp"
#include "coriolisTimesDqVector.hpp"
#include "dJTimesDqCalculator.hpp"
#include "gravityVector.hpp"
#include "inertiaMatrix.hpp"
#include "jacobianMatrix.hpp"
#include "jointTorquesSent.hpp"
#include "orientationErrorByPoses.hpp"
#include "posErrorByPoses.hpp"
#include "rotationErUnifiedAngleAxis.hpp"

namespace zakerimanesh {
CustomImpedanceNode::CustomImpedanceNode()
    : Node("impedance_node"), stop_control_loop_{false} {
  this->declare_parameter<std::vector<double>>("inertia", {0.5, 0.5, 0.5, 0.02, 0.02, 0.02});
  // this->declare_parameter<std::vector<double>>("inertia", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  this->declare_parameter<std::vector<double>>("stiffness", {tr_s, tr_s, tr_s, r_s, r_s, r_s});
  this->declare_parameter<std::vector<double>>("damping", {tr_d, tr_d, tr_d, r_d, r_d, r_d});

  // Retrieve parameter values. They can be overridden by YAML files or launch files.
  std::vector<double> inertia_values = this->get_parameter("inertia").as_double_array();
  std::vector<double> stiffness_values = this->get_parameter("stiffness").as_double_array();
  std::vector<double> damping_values = this->get_parameter("damping").as_double_array();

  // Fill in the diagonal elements.
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> inertia_values_mapped(inertia_values.data());
  inertia_.diagonal() = inertia_values_mapped;

  Eigen::Matrix<double, 6, 1> inertia_values_4inverse = inertia_values_mapped.cwiseInverse();
  inertia_inverse_.diagonal() = inertia_values_4inverse;

  Eigen::Map<const Eigen::Matrix<double, 6, 1>> stifness_values_mapped(stiffness_values.data());
  stiffness_.diagonal() = stifness_values_mapped;

  Eigen::Map<const Eigen::Matrix<double, 6, 1>> damping_values_mapped(damping_values.data());
  damping_.diagonal() = damping_values_mapped;

  this->declare_parameter<std::string>("robot_ip", "192.168.1.11");
  robot_ip_ = this->get_parameter("robot_ip").as_string();

  control_thread_ = std::thread(&CustomImpedanceNode::controlLoop, this);
}

CustomImpedanceNode::~CustomImpedanceNode() {
  stop_control_loop_ = true;
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

void CustomImpedanceNode::controlLoop() {
  // Pin this thread to core 2
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(2, &cpuset);
  pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

  RCLCPP_INFO(this->get_logger(), "Connecting to franka robot ...");

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

    std::array<double, 42> previous_jacobian =
        model.zeroJacobian(franka::Frame::kEndEffector, robot_initial_state);

    // desired pose = initial pose (position+orientation)
    Eigen::Affine3d end_effector_desired_pose;

    // model.zeroJacobian(franka::Frame::kEndEffector, robotOnlineState) gives
    // std::array<double,42> model.zeroJacobian(franka::Frame::kEndEffector,
    // robotOnlineState).data() return a pointer to the first element! so under the hood is like
    // double matrix[6][7] and using returned pointer it fills the matrix!
    end_effector_desired_pose.matrix() =
        convertArrayToEigenMatrix<4, 4>(robot_initial_state.O_T_EE);

    Eigen::Matrix<double, 6, 7> dot_jacobian_matrix = Eigen::Matrix<double, 6, 7>::Zero();

    // wrapper
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback;
    // impedance_control_call
    RCLCPP_INFO(this->get_logger(), "control loop ...");

    impedance_control_callback = [&](const franka::RobotState& robotOnlineState,
                                     franka::Duration /*duration*/) -> franka::Torques {
      if (stop_control_loop_) {
        // the special MotionFinished return halts the control loop immediately
        return franka::MotionFinished(franka::Torques{{0, 0, 0, 0, 0, 0, 0}});
      }

      // online end-effector pose (position+orientation)
      Eigen::Affine3d end_effector_online_pose;
      end_effector_online_pose.matrix() = convertArrayToEigenMatrix<4, 4>(robotOnlineState.O_T_EE);
      Eigen::Matrix<double, 3, 3> orientation_error_in_base_frame =
          end_effector_online_pose.rotation();

      // full end-ffector pose error (position+orientation) "the current pose away from the
      // desired"
      Eigen::Matrix<double, 6, 1> end_effector_full_pose_error;
      end_effector_full_pose_error
          << posErrorByPoses(end_effector_online_pose, end_effector_desired_pose),
          -orientation_error_in_base_frame *
              rotationErUnifiedAngleAxis(
                  orientationErrorByPoses(end_effector_online_pose, end_effector_desired_pose));

      // robot estimated external wrench (force and torque)
      Eigen::Matrix<double, 6, 1> ee_estimated_external_wrench;
      ee_estimated_external_wrench = convertArrayToEigenVector<6>(robotOnlineState.O_F_ext_hat_K);

      // robot Gravity (G)
      Eigen::Matrix<double, 7, 1> robot_gravity_vector = gravityVector(robotOnlineState, model);

      // robot inertia (M)
      Eigen::Matrix<double, 7, 7> robot_inertia_matrix = inertiaMatrix(robotOnlineState, model);

      // robot coriolis (C(q,dq)*dq)
      Eigen::Matrix<double, 7, 1> robot_coriolis_times_dq =
          coriolisTimesDqVector(robotOnlineState, model);

      // robot jacobian (J is 6*7)
      Eigen::Matrix<double, 6, 7> jacobian_matrix = jacobianMatrix(robotOnlineState, model);
      const Eigen::Matrix<double, 6, 7>& jacobian_matrix_alias = jacobian_matrix;

      // jacobian_transpose
      // instead of using jacobian_matrix.transpose() now JM.transpose(), to avoid copying
      Eigen::Matrix<double, 7, 6> jacobian_matrix_transpose = jacobian_matrix_alias.transpose();
      const Eigen::Matrix<double, 7, 6>& jacobian_matrix_transpose_alias =
          jacobian_matrix_transpose;

      // // 2) build the damped JJᵀ = J Jᵀ + λ²I
      // constexpr double λ = 0.001;
      Eigen::Matrix<double, 6, 6> JJt = jacobian_matrix_alias * jacobian_matrix_transpose_alias;
      // JJt.diagonal().array() += λ*λ;  // add λ² along the diagonal

      // pseudo-inverse jacobian (J+ is 7*6 = Jt(J*Jt)^-1) option1 : better (LDLT mthod)
      auto ldlt = JJt.ldlt();
      Eigen::Matrix<double, 6, 6> j_j_T_inverse =
          ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());
      Eigen::Matrix<double, 7, 6> jacobian_pseudo_matrix =
          jacobian_matrix_transpose_alias * j_j_T_inverse;

      // option2
      // Eigen::Matrix<double, 7, 6> jacobian_pseudo_matrix =
      //     jacobian_matrix_transpose * (jacobian_matrix * jacobian_matrix_transpose).inverse();

      // Computing on-line end-effector linear velocity (xd = Jacobian * djoint_velocities)
      Eigen::Matrix<double, 6, 1> ee_velocity =
          jacobian_matrix_alias * convertArrayToEigenVector<7>(robotOnlineState.dq);

      // dJ_Times_dq_calculator
      Eigen::Matrix<double, 6, 1> dJ_times_dq =
          dJTimesDqCalculator(robotOnlineState, model, dot_jacobian_matrix, previous_jacobian);

      // calculated torque
      // static Eigen::Matrix<double, 7, 1> last_tau = Eigen::Matrix<double, 7, 1>::Zero();
      // const double max_delta_tau = 0.001;  // Nm — conservative start

      Eigen::Matrix<double, 7, 1> tau_output = calculatedTorques(
          inertia_inverse_, stiffness_, damping_, end_effector_full_pose_error, ee_velocity,
          jacobian_matrix_transpose_alias, robot_inertia_matrix, robot_coriolis_times_dq,
          robot_gravity_vector, ee_estimated_external_wrench, jacobian_pseudo_matrix, dJ_times_dq);

      // // Torque rate clamping to avoid discontinuities
      // for (int i = 0; i < 7; ++i) {
      //   double delta = tau_output[i] - last_tau[i];
      //   if (delta > max_delta_tau) {
      //     tau_output[i] = last_tau[i] + max_delta_tau;
      //   } else if (delta < -max_delta_tau) {
      //     tau_output[i] = last_tau[i] - max_delta_tau;
      //   }
      // }

      //  // Add a static ramp factor to ease startup
      //   static double ramp_factor = 0.01;
      //   ramp_factor = std::min(ramp_factor + 0.1, 1.0);  // ramping from 0 to 1
      //   tau_output *= ramp_factor;

      //   // last_tau = tau_output;

      return jointTorquesSent(tau_output);
    };
    // ROS action 1KHZ
    RCLCPP_INFO(this->get_logger(), "Starting impedance control loop...");
    robot.control(impedance_control_callback);
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Franka Exception: %s", e.what());
  }
}
}  // namespace zakerimanesh
