#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "helper/FrankaRemote.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<zakerimanesh::FrankaRemote> node = std::make_shared<zakerimanesh::FrankaRemote>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*number of threads*/ 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
  return 0;
}
