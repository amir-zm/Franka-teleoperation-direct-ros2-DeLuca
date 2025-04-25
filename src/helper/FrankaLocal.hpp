#ifndef ZAKERIMANESH_Franka_Local_HPP
#define ZAKERIMANESH_Franka_Local_HPP

#include <Eigen/Dense>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

namespace zakerimanesh {
class FrankaLocal : public rclcpp::Node {
 private:
  void controlLoop();

  std::thread control_thread_;
  std::atomic<bool> stop_control_loop_;

  std::string robot_ip_;

  double tr_s = 121;
  double tr_d = 2 * std::sqrt(tr_s);

  double r_s = 9;
  double r_d = 2 * std::sqrt(r_s);

  Eigen::Matrix<double, 6, 6> stiffness_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> damping_ = Eigen::Matrix<double, 6, 6>::Zero();

 public:
  FrankaLocal();
  ~FrankaLocal();
};
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_Franka_Local_HPP