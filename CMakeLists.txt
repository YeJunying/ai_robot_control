cmake_minimum_required(VERSION 3.0.2)
project(ai_robot_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  behaviortree_cpp_v3
  ai_robot_waving
  ai_robot_tracking
)


add_action_files(DIRECTORY action FILES tracking.action)
generate_messages(DEPENDENCIES 
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    behaviortree_cpp_v3
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tracking_server
  src/tracking_server_node.cpp
  src/tracking_server.cpp
)
add_dependencies(tracking_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_server ${catkin_LIBRARIES})

add_executable(bh_tree_node 
  src/run_bh_tree.cpp
  src/tracking_client.cpp
  src/waving.cpp
  src/waving_detected.cpp
)
add_dependencies(bh_tree_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bh_tree_node ${catkin_LIBRARIES})


