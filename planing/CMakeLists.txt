cmake_minimum_required(VERSION 3.0.2)
project(dplanning)

add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(PACKAGE_DEPENDENCIES
  mavros
  mavros_msgs
  octomap_msgs
  tf
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  octomap_ros
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES planning
 CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
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

set( EXECUTABLE_SRC
src/planning_node.cpp
src/dplanning.cpp
src/planning_client.cpp
src/grid/Grid3D.cpp
src/grid/Node.cpp
src/path_planning/APF.cpp
src/tree/OctreeNode.cpp
src/tree/Octree.cpp
src/path_planning/RRTStar.cpp
src/path_planning/Astar.cpp
src/grid/updatable_priority_queue.cpp
)

add_executable(dplanning ${EXECUTABLE_SRC})
target_link_libraries(dplanning ${catkin_LIBRARIES})
