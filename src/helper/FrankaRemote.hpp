#ifndef ZAKERIMANESH_Franka_Remote_HPP
#define ZAKERIMANESH_Franka_Remote_HPP

#include <franka/robot_state.h>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <string>
#include <memory>
#include <functional>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>

namespace zakerimanesh {
class FrankaRemote : public rclcpp::Node {
  public:
  FrankaRemote();
  ~FrankaRemote();

  private:
  cpu_set_t cpuset1_;
  CPU_ZERO(&cpuset1_);
  CPU_SET(1, &cpuset1_);

  std::thread remote_control_thread_;
  std::thread remote_publish_thread_;
  std::atomic<bool> stop_control_loop_;
  std::string robot_ip_;
  Eigen::Matrix<double, 7, 1> stiffness_;
  Eigen::Matrix<double, 7, 1> damping_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subs_;  
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState msg_;
  franka::RobotState robotRemoteState_;
  rclcpp::QoS qos_settings_{5};

  void controlLoop();
  void remoteStateSubscription(const sensor_msgs::msg::JointState msg);
};
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_Franka_Remote_HPP