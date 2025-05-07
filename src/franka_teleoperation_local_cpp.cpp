
// // version 1

// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include "helper/FrankaLocal.hpp"

// int main(int argc, char* argv[]) {
//   rclcpp::init(argc, argv);
//   std::shared_ptr<zakerimanesh::FrankaLocal> node =
//   std::make_shared<zakerimanesh::FrankaLocal>(); rclcpp::executors::MultiThreadedExecutor
//   exec(rclcpp::ExecutorOptions(), /*number of threads*/ 2); exec.add_node(node); exec.spin();
//   rclcpp::shutdown();
//   return 0;
// }

// version 2

#include <chrono>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "helper/FrankaLocal.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // register
  rclcpp::on_shutdown([]() {
    RCLCPP_INFO(rclcpp::get_logger(""), "Caught shutdown — cleaning up!");
    // e.g. notify other threads, flush files, etc.
  });

  std::shared_ptr<zakerimanesh::FrankaLocal> node = std::make_shared<zakerimanesh::FrankaLocal>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*number of threads*/ 2);

  // 3) Spin in a detached thread
  std::thread spinner([&exec]() {
    // pin this spin thread to core 4
    cpu_set_t cpus4;
    CPU_ZERO(&cpus4);
    CPU_SET(4, &cpus4);
    pthread_setaffinity_np(pthread_self(), sizeof(cpus4), &cpus4);
    exec.spin();
  });
  spinner.detach();

  // 4) Block until someone hits Ctrl+C (or another thread calls shutdown())
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 5) Actually request shutdown — this will
  //    a) call your on_shutdown() hook
  //    b) cause exec.spin() to return
  rclcpp::shutdown();
  return 0;
}