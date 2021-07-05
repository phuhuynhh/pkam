cmake_minimum_required(VERSION 3.0.2)
project(dmapping)

add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  mavros
  mavros_msgs
  octomap_msgs
  tf
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_ros
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)


find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)


add_definitions(${PCL_DEFINITIONS})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES planning
 CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)



include_directories(
  include
  ${PCL_DEFINITIONS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
  ${OCTOMAP_INCLUDE_DIRS}}
)
include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)


# catkin_simple()

add_executable(mapping src/mapping_node.cpp)


target_link_libraries(mapping  ${PCL_LIBRARIES} ${catkin_LIBRARIES})