cmake_minimum_required(VERSION 3.8)
project(refbox_peer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED COMPONENTS)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(refbox_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message(${CMAKE_MODULE_PATH})

include(CheckCXXCompilerFlag)
		
message("protobuf_comm_FOUND: " ${protobuf_comm_FOUND})
find_package(RCLLRefbox COMPONENTS protobuf_comm llsf_msgs)

CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    message("-std=c++17")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    message("-std=c++0x")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Declare a C++ executable
add_executable(refbox_peer src/ros2-rcll_refbox_peer.cpp)
# add_dependencies(refbox_peer ${refbox_peer_EXPORTED_TARGETS})

list(APPEND _LIBRARIES "rcll_msgs")
list(APPEND Fawkes_INCLUDE_DIRS "/home/ryukoku/git/rcll-refbox/src/libs/")
list(APPEND Fawkes_INCLUDE_DIRS "/home/ryukoku/git/rcll-refbox/src/llsf_msgs/")
list(APPEND Fawkes_LIBRARIES "/home/ryukoku/git/rcll-refbox/lib/libllsf_protobuf_comm.so")
list(APPEND Fawkes_LIBRARIES "/home/ryukoku/git/rcll-refbox/lib/protobuf/libllsf_msgs.so")
list(APPEND Fawkes_LIBRARIES "/home/ryukoku/git/rcll-refbox/lib/protobuf/libllsf_log_msgs.so")
list(APPEND Fawkes_LIBRARIES "/home/ryukoku/git/rcll-refbox/lib/libmps_comm.so")
list(APPEND Fawkes_LIBRARIES "/usr/lib/x86_64-linux-gnu/libprotobuf.so")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Fawkes_INCLUDE_DIRS}
)
# Use Fawkes' protobuf_comm
set_target_properties(refbox_peer PROPERTIES
    INCLUDE_DIRECTORIES "${Fawkes_INCLUDE_DIRS};${catkin_INCLUDE_DIRS}"
    COMPILE_FLAGS "${Fawkes_CFLAGS} ${catkin_CFLAGS}"
    LINK_FLAGS "${Fawkes_LFLAGS}"
    LINK_LIBRARIES "${Fawkes_LIBRARIES};${catkin_LIBRARIES}"
    INSTALL_RPATH "${Fawkes_LIBRARY_DIRS}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE)
  message("Using Fawkes protobuf_comm")
  message("${Fawkes_INCLUDE_DIRS};${catkin_INCLUDE_DIRS}")

# link library to talker
ament_target_dependencies(refbox_peer
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  refbox_msgs
)

install(TARGETS
  refbox_peer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
