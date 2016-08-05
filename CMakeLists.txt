cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_object_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  rospy
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(floor_and_wall_removal src/floor_and_wall_removal.cpp)
target_link_libraries(floor_and_wall_removal ${catkin_LIBRARIES})

add_executable(object_clustering src/object_clustering.cpp)
target_link_libraries(object_clustering ${catkin_LIBRARIES})

add_executable(PCL_transform src/PCL_transform.cpp)
target_link_libraries(PCL_transform ${catkin_LIBRARIES})
