cmake_minimum_required(VERSION 3.8)
project(ros2-rcll_refbox_peer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# create executable talker
add_executable(refboxpeer src/ros2-rcll_refbox_peer.cpp)
# link library to talker
ament_target_dependencies(refboxpeer
  rclcpp
  std_msgs
)

install(TARGETS
  refboxpeer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
