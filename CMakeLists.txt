cmake_minimum_required(VERSION 3.8)
project(franka_teleoperation)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Franka 0.14.1 REQUIRED)     
find_package(franka_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(franka_teleoperation
  src/franka_teleoperation_cpp.cpp
  src/helper/FrankaLocal.cpp
  src/helper/FrankaRemote.cpp
  /home/mobilerobot/franka_ws/src/libfranka/examples/examples_common.cpp)

ament_target_dependencies(
  franka_teleoperation
  rclcpp
  franka_msgs
  sensor_msgs
  std_msgs
  Eigen3
)
target_link_libraries(franka_teleoperation
  Franka::Franka
)

target_link_libraries(franka_teleoperation
  Franka::Franka
)

# install targets
install(
  TARGETS
  franka_teleoperation
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install launch & config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
