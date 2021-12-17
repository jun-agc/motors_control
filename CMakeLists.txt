cmake_minimum_required(VERSION 2.8.3)
project(motors_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES motors_control
    CATKIN_DEPENDS roscpp rospy std_msgs 
    DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_test_node src/motor_test.cpp)
add_dependencies(motor_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_test_node ${catkin_LIBRARIES})
