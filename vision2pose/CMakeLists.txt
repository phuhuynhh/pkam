cmake_minimum_required(VERSION 3.0.2)
project(vision2pose)

add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  roscpp
  geometry_msgs
  nav_msgs
  rospy
  tf
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES vision2pose
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
    ${catkin_INCLUDE_DIRS}
)
#############
## Install ##
#############
link_libraries(${catkin_LIBRARIES})
add_executable(vision2pose src/vision_node.cpp)
target_link_libraries(vision2pose ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision2pose.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
