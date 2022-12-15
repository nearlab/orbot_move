cmake_minimum_required (VERSION 3.5)
project (orbot_move)
set (CMAKE_CXX_STANDARD 11)


include_directories(${Eigen_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  kinova_driver
  actionlib
  message_generation
  kinova_msgs
)
catkin_package(CATKIN_DEPENDS roscpp actionlib)
include_directories(
${catkin_INCLUDE_DIRS}
)
add_executable(orbot_move src/orbot_move.cpp)
target_link_libraries (orbot_move serial ${catkin_LIBRARIES})
