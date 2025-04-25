cmake_minimum_required(VERSION 3.8)
project(franka_teleoperation)

# Enable C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Franka CONFIG REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.14.1 REQUIRED)

# include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# executable
add_executable(franka_local_node
  src/franka_teleoperation_local_cpp.cpp
  src/helper/frankaLocal.cpp
  /home/mobilerobot/franka_ws/src/libfranka/examples/examples_common.cpp
)

ament_target_dependencies(franka_local_node
  rclcpp
  franka_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(franka_local_node
  Franka::Franka
)

# install targets
install(TARGETS
  franka_local_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install launch and config directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
