cmake_minimum_required(VERSION 2.8.3)
project(denso_gazebo_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_service_files(
  FILES
  GetJointsValue.srv
  Pose.srv
  SetJointsValue.srv
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
