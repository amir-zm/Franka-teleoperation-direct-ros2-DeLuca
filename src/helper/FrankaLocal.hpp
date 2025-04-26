#ifndef ZAKERIMANESH_Franka_Local_HPP
#define ZAKERIMANESH_Franka_Local_HPP

#include <Eigen/Dense>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

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
  void controlLoop();
};
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_Franka_Local_HPP