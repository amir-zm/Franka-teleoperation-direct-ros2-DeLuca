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
#include "helper/FrankaRemote.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto local_node  = std::make_shared<zakerimanesh::FrankaLocal>();
  auto remote_node = std::make_shared<zakerimanesh::FrankaRemote>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(local_node);
  exec.add_node(remote_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
