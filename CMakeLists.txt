cmake_minimum_required (VERSION 3.5)
if(NOT TARGET Orbot_utils)
project(orbot_utils)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/..)

file(GLOB ORBOT_UTILS_SOURCES ${PROJECT_SOURCE_DIR}/src/orbot_utils*)
add_library(orbot_utils SHARED ${ORBOT_UTILS_SOURCES})
endif()

project (orbot_move)
SET(GCC_FLAGS "-g")
add_definitions(${GCC_FLAGS})
find_package(Eigen3 REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  kinova_driver
)
catkin_package()
include_directories(
${catkin_INCLUDE_DIRS}
)
add_executable(orbot_move src/orbot_move.cpp)
target_link_libraries (orbot_move orbot_utils serial ${catkin_LIBRARIES})
