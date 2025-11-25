cmake_minimum_required(VERSION 3.0.2)
project(rover_teleop)

# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_core
  pluginlib
  costmap_2d
  tf
  angles
  base_local_planner
)

# Declare a catkin package
catkin_package(
  LIBRARIES rover_teleop 
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_core pluginlib costmap_2d tf angles base_local_planner
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add executable for realsense_to_pointcloud
add_executable(realsense_to_pointcloud scripts/realsense_to_pointcloud.cpp)

# Link libraries to the realsense_to_pointcloud executable
target_link_libraries(realsense_to_pointcloud
  ${catkin_LIBRARIES}
)
