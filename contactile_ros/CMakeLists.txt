cmake_minimum_required(VERSION 3.8)
project(contactile_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(contactile_msgs REQUIRED)

INCLUDE_DIRECTORIES(
  lib
  include
)
link_directories(lib)

add_executable(contactile_node src/contactile_node.cpp src/contactile_main.cpp)
target_link_libraries(contactile_node libPTSDK.a)
ament_target_dependencies(contactile_node rclcpp rclcpp_action contactile_msgs)

# INSTALL
install(TARGETS
contactile_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
