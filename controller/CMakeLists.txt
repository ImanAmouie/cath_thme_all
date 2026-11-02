cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(controller_node src/controller.cpp)

ament_target_dependencies(
  controller_node
  rclcpp
  geometry_msgs
  turtlesim
)

install(
  TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()