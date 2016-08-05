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
  message_generation
)

add_message_files(
  FILES
  object_loc.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
