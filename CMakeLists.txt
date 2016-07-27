cmake_minimum_required(VERSION 2.8.3)
project(trajectory_ros_control)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface 
  roscpp
  std_msgs
  sensor_msgs
  controller_manager
  hardware_interface
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs controller_manager hardware_interface moveit_ros_planning_interface 
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(imau_control_node src/arm_interface_node.cpp src/Arm_interface.cpp src/Arm_interface_sim.cpp)
add_dependencies(imau_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(imau_control_node ${catkin_LIBRARIES})
