cmake_minimum_required(VERSION 2.8.3)
project(denso_gazebo_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  Contact.msg
  ContactArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
