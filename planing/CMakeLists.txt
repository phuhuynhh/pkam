cmake_minimum_required(VERSION 3.0.2)
project(dplanning)

add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  roscpp
  tf
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES planning
 CATKIN_DEPENDS mavros mavros_msgs roscpp rospy tf
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


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  	${catkin_INCLUDE_DIRS}
)

add_executable(dplanning src/planning_node.cpp src/dplanning.cpp src/planning_client.cpp)
target_link_libraries(dplanning ${catkin_LIBRARIES})