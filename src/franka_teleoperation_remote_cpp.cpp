#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "helper/FrankaRemote.hpp"


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<zakerimanesh::FrankaRemote> node =
      std::make_shared<zakerimanesh::FrankaRemote>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
