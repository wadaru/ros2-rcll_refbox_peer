cmake_minimum_required(VERSION 3.5)

project(refbox_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AgentTask.msg"
  "msg/LightSpec.msg"
  "msg/Order.msg"
  "msg/BeaconSignal.msg"
  "msg/MachineInfo.msg"
  "msg/ProductColor.msg"
  "msg/BufferStation.msg"
  "msg/Machine.msg"
  "msg/Retrieve.msg"
  "msg/Deliver.msg"
  "msg/MachineReportEntryBTR.msg"
  "msg/RingInfo.msg"
  "msg/ExplorationInfo.msg"
  "msg/MachineReportEntry.msg"
  "msg/Ring.msg"
  "msg/ExplorationSignal.msg"
  "msg/MachineReportInfo.msg"
  "msg/Route.msg"
  "msg/ExplorationZone.msg"
  "msg/Move.msg"
  "msg/Team.msg"
  "msg/ExploreWaypoint.msg"
  "msg/NavigationRoutes.msg"
  "msg/Time.msg"
  "msg/GameState.msg"
  "msg/OrderInfo.msg"
  "msg/WorkpieceDescription.msg"
  "srv/SendAgentTask.srv"
  "srv/SendMachineReportBTR.srv"
  "srv/SendPrepareMachine.srv"
  "srv/SendBeaconSignal.srv"
  "srv/SendMachineReport.srv"
  DEPENDENCIES std_msgs #Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
  DEPENDENCIES geometry_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
