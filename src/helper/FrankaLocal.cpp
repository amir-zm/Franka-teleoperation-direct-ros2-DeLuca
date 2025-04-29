
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
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <thread>
#include <vector>

#include "FrankaLocal.hpp"
#include "convertArrayToEigenMatrix.hpp"
#include "convertArrayToEigenVector.hpp"
#include "coriolisTimesDqVector.hpp"
#include "jacobianMatrix.hpp"
#include "jointTorquesSent.hpp"
#include "localCalculatedTorques.hpp"
#include "orientationErrorByPoses.hpp"
#include "posErrorByPoses.hpp"
#include "rotationErUnifiedAngleAxis.hpp"

namespace zakerimanesh {
FrankaLocal::FrankaLocal() : Node("franka_teleoperation_local_node"), stop_control_loop_{false} {
  this->declare_parameter<std::string>("robot_ip", "192.168.1.11");
  robot_ip_ = this->get_parameter("robot_ip").as_string();

  this->declare_parameter<std::vector<double>>("stiffness",
                                               {121.0, 121.0, 121.0, 25.0, 25.0, 25.0});
  auto stiffness_raw = this->get_parameter("stiffness").as_double_array();
  stiffness_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(stiffness_raw.data());

  this->declare_parameter<std::vector<double>>("damping", {22.0, 22.0, 22.0, 10.0, 10.0, 10.0});
  auto damping_raw = this->get_parameter("damping").as_double_array();
  damping_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(damping_raw.data());

  qos_settings_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("local_joint_states", qos_settings_);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&FrankaLocal::localStatePublishFrequency, this));  // timer = is necessary!!

  local_control_thread_ = std::thread(&FrankaLocal::controlLoop, this);

  msg_.name = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
               "fr3_joint5", "fr3_joint6", "fr3_joint7"};
}

FrankaLocal::~FrankaLocal() {
  stop_control_loop_ = true;
  if (local_control_thread_.joinable()) {
    local_control_thread_.join();
  }
}

void FrankaLocal::localStatePublishFrequency() {
  // Pin this thread to core 4
  CPU_ZERO(&cpuset4_);
  CPU_SET(4, &cpuset4_);
  pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset4_);
  {
    std::lock_guard<std::mutex> publishLock(robot_state_pub_mutex_);
    msg_.header.stamp = this->now();
    msg_.position = std::vector<double>(robotOnlineState_.q.begin(), robotOnlineState_.q.end());
    msg_.velocity = std::vector<double>(robotOnlineState_.dq.begin(), robotOnlineState_.dq.end());
    // msg_.effort =
    //     std::vector<double>(robotOnlineState_.tau_J.begin(), robotOnlineState_.tau_J.end());
  }

  joint_state_pub_->publish(msg_);
}

void FrankaLocal::controlLoop() {
  // Pin this thread to core 2
  CPU_ZERO(&cpuset2_);
  CPU_SET(2, &cpuset2_);
  pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset2_);

  RCLCPP_INFO(this->get_logger(), "Connecting to franka local robot ...");

  try {
    franka::Robot robot(robot_ip_);
    setDefaultBehavior(robot);

    franka::Model model = robot.loadModel();

    double torque_thresholds = 80;
    double force_thresholds = 80;

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

    // Fill in the diagonal elements
    Eigen::Matrix<double, 6, 6> stiffness = Eigen::Matrix<double, 6, 6>::Zero();
    stiffness.diagonal() = stiffness_;

    Eigen::Matrix<double, 6, 6> damping = Eigen::Matrix<double, 6, 6>::Zero();
    damping.diagonal() = damping_;

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
      // publishing local joints
      {
        std::lock_guard<std::mutex> publishLock(robot_state_pub_mutex_);
        robotOnlineState_ = robotOnlineState;
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
          localCalculatedTorques(stiffness, damping, end_effector_full_pose_error, ee_velocity,
                                 jacobian_matrix_transpose_alias, robot_coriolis_times_dq);

      return jointTorquesSent(tau_output);
    };
    // ROS action 1KHZ
    RCLCPP_INFO(this->get_logger(), "Starting local impedance control loop...");
    robot.control(impedance_control_callback, true);
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Franka Exception: %s", e.what());
  }
}
}  // namespace zakerimanesh
