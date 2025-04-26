#ifndef ZAKERIMANESH_Franka_Local_HPP
#define ZAKERIMANESH_Franka_Local_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <string>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>

namespace zakerimanesh {
class FrankaLocal : public rclcpp::Node {
  public:
  FrankaLocal();
  ~FrankaLocal();

 private:
  std::thread control_thread_;
  std::atomic<bool> stop_control_loop_;
  std::string robot_ip_;
  Eigen::Matrix<double, 6, 1> stiffness_;
  Eigen::Matrix<double, 6, 1> damping_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("master_joint_states", 10);
  


  void controlLoop();
};
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_Franka_Local_HPP