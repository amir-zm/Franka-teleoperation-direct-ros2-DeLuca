#ifndef ZAKERIMANESH_Franka_Local_HPP
#define ZAKERIMANESH_Franka_Local_HPP

#include <franka/robot_state.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <string>
#include <memory>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>

namespace zakerimanesh {
class FrankaLocal : public rclcpp::Node {
  public:
  FrankaLocal();
  ~FrankaLocal();

 private:
  std::mutex robot_state_pub_mutex_;
  std::thread local_control_thread_;

  std::atomic<bool> stop_control_loop_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;  
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState msg_;

  franka::RobotState robotOnlineState_; 

  rclcpp::QoS qos_settings_{5};

  std::string robot_ip_;

  Eigen::Matrix<double, 6, 1> inertia_vector_;
  Eigen::Matrix<double, 6, 1> stiffness_vector_;
  Eigen::Matrix<double, 6, 1> damping_vector_;

  Eigen::Matrix<double, 6, 6> inertia_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> inertia_matrix_inverse_ = Eigen::Matrix<double, 6, 6>::Zero();

  cpu_set_t cpuset2_;
  cpu_set_t cpuset4_;

  void controlLoop();
  void localStatePublishFrequency();
};
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_Franka_Local_HPP