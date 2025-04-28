// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include "helper/FrankaLocal.hpp"

// int main(int argc, char* argv[]) {
//   rclcpp::init(argc, argv);
//   std::shared_ptr<zakerimanesh::FrankaLocal> node =
//       std::make_shared<zakerimanesh::FrankaLocal>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "helper/FrankaLocal.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<zakerimanesh::FrankaLocal> node =
      std::make_shared<zakerimanesh::FrankaLocal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
