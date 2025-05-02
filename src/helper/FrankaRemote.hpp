#ifndef ZAKERIMANESH_Franka_Remote_HPP
#define ZAKERIMANESH_Franka_Remote_HPP

#include <franka/robot_state.h>

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <thread>

namespace zakerimanesh {
class FrankaRemote : public rclcpp::Node {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrankaRemote();
  ~FrankaRemote();

 private:
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
  std::mutex robot_state_sub_mutex_;

  cpu_set_t cpuset3_;
  cpu_set_t cpuset1_;

  void controlLoop();
  void remoteStateSubscription(const sensor_msgs::msg::JointState msg);
};
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_Franka_Remote_HPP