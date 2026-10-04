cmake_minimum_required(VERSION 3.0.2)
project(fast_livo2_assist_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(assist_planner_node src/assist_planner_node.cpp)
target_link_libraries(assist_planner_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/assist_video_overlay.py
  scripts/publish_intent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
