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

namespace zakerimanesh {
class FrankaLocal : public rclcpp::Node {
  public:
  FrankaLocal();
  ~FrankaLocal();

 private:
  std::thread local_control_thread_;
  std::thread local_publish_thread_;
  std::atomic<bool> stop_control_loop_;
  std::string robot_ip_;
  Eigen::Matrix<double, 6, 1> stiffness_;
  Eigen::Matrix<double, 6, 1> damping_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;  
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState msg_;
  rclcpp::QoS qos_settings_{5};

  cpu_set_t cpuset2_;



  void controlLoop();
  void localStatePublishFrequency(const franka::RobotState& robotOnlineState);
};
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_Franka_Local_HPP