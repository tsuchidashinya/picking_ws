cmake_minimum_required(VERSION 2.8.3)
project(denso_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  rospy
  std_srvs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    roscpp
    tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(model_tf_publisher
  src/model_tf_publisher.cpp
  src/model_tf_publisher_node.cpp)
target_link_libraries(model_tf_publisher
  ${catkin_LIBRARIES})

#############
## Install ##
#############

set(EXEC_FILES
  model_tf_publisher
  )

foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(PYTHON_SCRIPTS
  scripts/driver.py
  scripts/listener.py
  )

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

set(DIRECTORIES
  ./config/
  ./launch/
  ./urdf/
  ./worlds/
  )

# other files
foreach(dirs ${DIRECTORIES})
   install(DIRECTORY ${dirs}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  set(DIRECTORIES
    ./launch/
    )

  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(${dirs})
  endforeach()
endif()
