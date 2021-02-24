cmake_minimum_required(VERSION 2.8.3)
project(denso_gazebo_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    angles
    control_toolbox
    denso_gazebo_msgs
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
    pluginlib
    roscpp
    urdf
)

find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
find_package(gazebo REQUIRED)

find_package(Boost REQUIRED system filesystem regex thread)
find_package(Protobuf REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
    roscpp
  DEPENDS gazebo
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(gazebo_grasp_contacts_bridge_node
  src/gazebo_grasp_contacts_bridge.cpp
  src/gazebo_grasp_contacts_bridge_node.cpp
)

target_link_libraries(gazebo_grasp_contacts_bridge_node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

#[[add_library(${PROJECT_NAME}
  src/denso_robot_hw_sim.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} gazebo_grasp_contacts_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

set(DIRECTORIES
  ./config/
  ./launch/
)

foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()]]
