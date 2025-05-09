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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "FrankaRemote.hpp"
#include "convertArrayToEigenVector.hpp"
#include "coriolisTimesDqVector.hpp"
#include "jointTorquesSent.hpp"
#include "remoteCalculatedTorques.hpp"

namespace zakerimanesh {
FrankaRemote::FrankaRemote() : Node("franka_teleoperation_remote_node"), stop_control_loop_{false} {
  this->declare_parameter<std::string>("robot_ip", "192.168.1.12");
  robot_ip_ = this->get_parameter("robot_ip").as_string();

  this->declare_parameter<std::vector<double>>("stiffness",
                                               {25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0});
  auto stiffness_raw = this->get_parameter("stiffness").as_double_array();
  stiffness_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(stiffness_raw.data());

  this->declare_parameter<std::vector<double>>("damping",
                                               {18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0});
  auto damping_raw = this->get_parameter("damping").as_double_array();
  damping_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(damping_raw.data());

  qos_settings_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  remote_control_thread_ = std::thread(&FrankaRemote::controlLoop, this);

  joint_state_subs_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "local_joint_states", qos_settings_,
      std::bind(&FrankaRemote::remoteStateSubscription, this, std::placeholders::_1));

  msg_.name = {"remote_fr3_joint1", "remote_fr3_joint2", "remote_fr3_joint3", "remote_fr3_joint4",
               "remote_fr3_joint5", "remote_fr3_joint6", "remote_fr3_joint7"};

  msg_.position.resize(7);
  msg_.velocity.resize(7);
  // msg_.effort.resize(7);
}

FrankaRemote::~FrankaRemote() {
  stop_control_loop_ = true;
  if (remote_control_thread_.joinable()) {
    remote_control_thread_.join();
  }
}

void FrankaRemote::remoteStateSubscription(const sensor_msgs::msg::JointState msg) {
  // // Pin this thread to core 1
  // cpu_set_t cpuset1;
  // CPU_ZERO(&cpuset1);
  // CPU_SET(1, &cpuset1);
  // pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset1);
  {
    std::lock_guard<std::mutex> subLock(robot_state_sub_mutex_);
    msg_.header.stamp = this->now();
    std::copy(msg.position.begin(), msg.position.end(), msg_.position.begin());
    std::copy(msg.velocity.begin(), msg.velocity.end(), msg_.velocity.begin());
  }
}

void FrankaRemote::controlLoop() {
  // CPU_ZERO(&cpuset3_);
  // CPU_SET(3, &cpuset3_);
  // // Pin this thread to core 3
  // pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset3_);

  RCLCPP_INFO(this->get_logger(), "Connecting to franka remote robot ...");

  try {
    franka::Robot robot(robot_ip_);
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

    double torque_thresholds = 30;
    double force_thresholds = 30;

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

    Eigen::Matrix<double, 7, 1> desired_joints_positions =
        convertArrayToEigenVector(robot_initial_state.q);
    Eigen::Matrix<double, 7, 1> desired_joints_velocities =
        convertArrayToEigenVector(robot_initial_state.dq);
    Eigen::Matrix<double, 7, 1> online_joints_positions_error;
    Eigen::Matrix<double, 7, 1> online_joints_velocities_error;
    Eigen::Matrix<double, 7, 1> robot_coriolis_times_dq;
    Eigen::Matrix<double, 7, 1> tau_output;
    Eigen::Matrix<double, 7, 7> stiffness = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 7, 7> damping = Eigen::Matrix<double, 7, 7>::Zero();

    // Fill in the diagonal elements
    stiffness.diagonal() = stiffness_;
    damping.diagonal() = damping_;

    // wrapper
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback;
    // impedance_control_call
    RCLCPP_INFO(this->get_logger(), "jointspace control loop ...");

    impedance_control_callback = [&](const franka::RobotState& robotOnlineState,
                                     franka::Duration /*duration*/) -> franka::Torques {
      if (stop_control_loop_) {
        // the special MotionFinished return halts the control loop immediately
        return franka::MotionFinished(franka::Torques{{0, 0, 0, 0, 0, 0, 0}});
      }

      {
        std::lock_guard<std::mutex> subLock(robot_state_sub_mutex_);
        desired_joints_positions = Eigen::Map<Eigen::Matrix<double, 7, 1>>(msg_.position.data());
        desired_joints_velocities = Eigen::Map<Eigen::Matrix<double, 7, 1>>(msg_.velocity.data());
      }

      // online joints positions
      online_joints_positions_error =
          convertArrayToEigenVector<7>(robotOnlineState.q) - desired_joints_positions;
      online_joints_velocities_error =
          convertArrayToEigenVector<7>(robotOnlineState.dq) - desired_joints_velocities;

      // robot coriolis (C(q,dq)*dq)
      robot_coriolis_times_dq = coriolisTimesDqVector(robotOnlineState, model);

      // calculated torque
      tau_output = remoteCalculatedTorques(stiffness, damping, online_joints_positions_error,
                                           online_joints_velocities_error, robot_coriolis_times_dq);

      return jointTorquesSent(tau_output);
    };
    // ROS action 1KHZ
    RCLCPP_INFO(this->get_logger(), "Starting remote impedance control loop...");
    robot.control(impedance_control_callback, true);
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Franka Exception: %s", e.what());
  }
}
}  // namespace zakerimanesh