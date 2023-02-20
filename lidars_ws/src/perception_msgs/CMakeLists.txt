cmake_minimum_required(VERSION 3.0.2)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Obstacle.msg
  ObstacleArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

