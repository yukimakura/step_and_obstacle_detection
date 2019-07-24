cmake_minimum_required(VERSION 2.8.3)
project(plane_detect_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plane_detect_pcl
  CATKIN_DEPENDS  roscpp rospy sensor_msgs std_msgs
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(plane_detect_node src/plane_detect_node.cpp src/plane_detect.cpp)
add_executable(plane_removal_node src/plane_removal_node.cpp src/plane_removal.cpp)
add_executable(obstacle_detect_node src/obstacle_detect_node.cpp src/obstacle_detect.cpp)
add_executable(downsampling_node src/downsampling_node.cpp src/downsampling.cpp)
add_executable(gpu_euclidean_cluster_node src/gpu_euclidean_cluster_node.cpp src/gpu_euclidean_cluster.cpp)



target_link_libraries(plane_detect_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(plane_removal_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(obstacle_detect_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(downsampling_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(gpu_euclidean_cluster_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
