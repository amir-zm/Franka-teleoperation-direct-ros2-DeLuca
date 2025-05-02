
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
#include <sys/mman.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
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
#include "dJTimesDqCalculator.hpp"
#include "inertiaMatrix.hpp"
#include "jacobianMatrix.hpp"
#include "jointTorquesSent.hpp"
#include "localCalculatedTorques.hpp"
#include "orientationErrorByPoses.hpp"
#include "posErrorByPoses.hpp"
#include "rotationErUnifiedAngleAxis.hpp"
#include "templateClamp.hpp"

namespace zakerimanesh {
FrankaLocal::FrankaLocal() : Node("franka_teleoperation_local_node"), stop_control_loop_{false} {
  mlockall(MCL_CURRENT | MCL_FUTURE);

  this->declare_parameter<std::string>("robot_ip", "192.168.1.11");
  robot_ip_ = this->get_parameter("robot_ip").as_string();

  this->declare_parameter<std::vector<double>>("inertia", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  auto inertia_raw = this->get_parameter("inertia").as_double_array();
  inertia_vector_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(inertia_raw.data());

  this->declare_parameter<std::vector<double>>("stiffness",
                                               {121.0, 121.0, 121.0, 25.0, 25.0, 25.0});
  auto stiffness_raw = this->get_parameter("stiffness").as_double_array();
  stiffness_vector_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(stiffness_raw.data());

  this->declare_parameter<std::vector<double>>("damping", {22.0, 22.0, 22.0, 10.0, 10.0, 10.0});
  auto damping_raw = this->get_parameter("damping").as_double_array();
  damping_vector_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(damping_raw.data());

  this->inertia_matrix_.diagonal() = inertia_vector_;
  stiffness_matrix_.diagonal() = stiffness_vector_;
  damping_matrix_.diagonal() = damping_vector_;

  inertia_matrix_inverse_.diagonal() = inertia_vector_.cwiseInverse();

  qos_settings_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("local_joint_states", qos_settings_);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&FrankaLocal::localStatePublishFrequency, this));  // timer = is necessary!!

  local_control_thread_ = std::thread(&FrankaLocal::controlLoop, this);

  msg_.name = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
               "fr3_joint5", "fr3_joint6", "fr3_joint7"};

  msg_.position.resize(7);
  msg_.velocity.resize(7);
  msg_.effort.resize(7);
}

FrankaLocal::~FrankaLocal() {
  stop_control_loop_ = true;
  if (local_control_thread_.joinable()) {
    local_control_thread_.join();
  }
}

void FrankaLocal::localStatePublishFrequency() {
  // // Pin this thread to core 4
  // CPU_ZERO(&cpuset4_);
  // CPU_SET(4, &cpuset4_);
  // pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset4_);
  {
    std::lock_guard<std::mutex> publishLock(robot_state_pub_mutex_);
    msg_.header.stamp = this->now();
    std::copy(robotOnlineState_.q.begin(), robotOnlineState_.q.end(), msg_.position.begin());
    std::copy(robotOnlineState_.dq.begin(), robotOnlineState_.dq.end(), msg_.velocity.begin());
    std::copy(tau_output_.begin(), tau_output_.end(), msg_.effort.begin());
  }

  joint_state_pub_->publish(msg_);
}

void FrankaLocal::controlLoop() {
  // struct sched_param p{ .sched_priority = 99};
  // // Pin this thread to core 2
  // CPU_ZERO(&cpuset2_);
  // CPU_SET(2, &cpuset2_);
  // pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset2_);
  // pthread_setschedparam(pthread_self(), SCHED_FIFO, &p);

  RCLCPP_INFO(this->get_logger(), "Connecting to franka local robot ...");

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

    // desired pose = initial pose (position+orientation)
    Eigen::Affine3d end_effector_desired_pose;
    end_effector_desired_pose.matrix() =
        convertArrayToEigenMatrix<4, 4>(robot_initial_state.O_T_EE);

    Eigen::Affine3d end_effector_online_pose;
    Eigen::Matrix<double, 6, 1> end_effector_full_pose_error;
    Eigen::Matrix<double, 7, 1> robot_coriolis_times_dq;
    Eigen::Matrix<double, 7, 6> psuedo_jacobian_matrix;
    Eigen::Matrix<double, 7, 6> jacobian_matrix_transpose;
    Eigen::Matrix<double, 7, 6>& jacobian_matrix_transpose_alias = jacobian_matrix_transpose;
    Eigen::Matrix<double, 6, 1> ee_velocity;
    Eigen::Matrix<double, 6, 7> dot_jacobian_matrix = Eigen::Matrix<double, 6, 7>::Zero();
    Eigen::Matrix<double, 6, 7> previous_jacobian_matrix = convertArrayToEigenMatrix<6, 7>(
        model.zeroJacobian(franka::Frame::kEndEffector, robot_initial_state));
    Eigen::Matrix<double, 6, 7> current_jacobian_matrix = Eigen::Matrix<double, 6, 7>::Zero();
    Eigen::Matrix<double, 7, 1> joint_velocities;
    Eigen::Matrix<double, 7, 7> robot_inertia_matrix;
    Eigen::Matrix<double, 6, 1> dJ_times_dq;
    Eigen::Matrix<double, 6, 6> JJt;
    Eigen::Matrix<double, 6, 7> JJt_inverse_J;
    Eigen::Matrix<double, 3, 3> orientation_error_in_base_frame;
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> LDLT_instance;
    Eigen::Matrix<double, 6, 6> identity_matrix_6by6 = Eigen::Matrix<double, 6, 6>::Identity();

    tau_output_ = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> tau_output;
    // wrapper
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback;
    // impedance_control_call
    RCLCPP_INFO(this->get_logger(), "local control loop ...");

    impedance_control_callback = [&](const franka::RobotState& robotOnlineState,
                                     franka::Duration duration) -> franka::Torques {
      if (stop_control_loop_) {
        // the special MotionFinished return halts the control loop immediately
        return franka::MotionFinished(franka::Torques{{0, 0, 0, 0, 0, 0, 0}});
      }

      // }
      // robot inertia matrix (7*7)
      robot_inertia_matrix = inertiaMatrix(robotOnlineState, model);

      // online end-effector pose (position+orientation)
      end_effector_online_pose.matrix() = convertArrayToEigenMatrix<4, 4>(robotOnlineState.O_T_EE);
      orientation_error_in_base_frame = end_effector_online_pose.rotation();

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
      current_jacobian_matrix = jacobianMatrix(robotOnlineState, model);

      // jacobian_transpose
      jacobian_matrix_transpose = current_jacobian_matrix.transpose();

      // Computing on-line end-effector linear velocity (xd = Jacobian * djoint_velocities)
      joint_velocities = convertArrayToEigenVector<7>(robotOnlineState.dq);
      ee_velocity = current_jacobian_matrix * joint_velocities;

      // pseudo-inverse jacobian
      JJt = current_jacobian_matrix * jacobian_matrix_transpose_alias;
      JJt = 0.5 * (JJt + JJt.transpose());
      LDLT_instance.compute(JJt.selfadjointView<Eigen::Lower>());
      assert(LDLT_instance.info() == Eigen::Success);
      JJt_inverse_J = LDLT_instance.solve(current_jacobian_matrix);
      psuedo_jacobian_matrix = JJt_inverse_J.transpose();

      dJ_times_dq = dJTimesDqCalculator(previous_jacobian_matrix, current_jacobian_matrix,
                                        dot_jacobian_matrix, joint_velocities, duration);

      // calculated torque
      tau_output = localCalculatedTorques(
          inertia_matrix_inverse_, stiffness_matrix_, damping_matrix_, end_effector_full_pose_error,
          ee_velocity, jacobian_matrix_transpose_alias, psuedo_jacobian_matrix,
          robot_coriolis_times_dq, robot_inertia_matrix, dJ_times_dq);

      {
        std::lock_guard<std::mutex> publishLock(robot_state_pub_mutex_);
        tau_output_ = tau_output;
        robotOnlineState_ = robotOnlineState;
      }

      for (int i = 0; i < 7; ++i) {
        tau_output[i] = templateClamp<double>(tau_output[i], -30.0, 30.0);
      }

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
