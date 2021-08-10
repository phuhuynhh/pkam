cmake_minimum_required(VERSION 2.8.3)
project(ewok_ring_buffer)

# set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(-std=c++11 -g -O3 -ftree-vectorize -march=native)

find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    pcl_conversions
    pcl_msgs
    pcl_ros
    message_filters
    std_msgs
    geometry_msgs
    visualization_msgs
    mav_trajectory_generation
    mav_trajectory_generation_ros
)
# find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
catkin_package()

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR}) 
include_directories(${PROJECT_SOURCE_DIR}/include)
# ${OCTOMAP_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})

link_libraries(${catkin_LIBRARIES} ${OMPL_LIBRARY_DIRS})

add_executable(realtime_example src/realtime_example.cpp)
target_link_libraries(realtime_example ${PCL_LIBRARIES} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})


# install(TARGETS realtime_example RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/${PROJECT_NAME})

# catkin_simple()

# cs_add_executable(ring_buffer_example src/ring_buffer_example.cpp)
# cs_add_executable(tum_rgbd_ring_buffer_example src/tum_rgbd_ring_buffer_example.cpp)
# target_link_libraries(tum_rgbd_ring_buffer_example ${OCTOMAP_LIBRARIES})



# catkin_add_gtest(test_ring_buffer_base test/ring-buffer-base-test.cpp)

# cs_install()
# cs_export()
