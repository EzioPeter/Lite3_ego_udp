cmake_minimum_required(VERSION 3.0.2)
project(lite3_ego_udp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  tf2_eigen
  quadrotor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf tf2_eigen quadrotor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(sender SHARED src/sender.cpp)
add_executable(lite3_ego_udp_node src/lite3_command.cpp)
target_link_libraries(lite3_ego_udp_node
  ${catkin_LIBRARIES}
  sender
)
