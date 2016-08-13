cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_object_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  rospy
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_ros
  tf
  object_detecter_2d
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(PCL_ground_removal src/PCL_ground_removal.cpp)
target_link_libraries(PCL_ground_removal ${catkin_LIBRARIES})

add_executable(PCL_object_clustering src/PCL_object_clustering.cpp)
target_link_libraries(PCL_object_clustering ${catkin_LIBRARIES})
