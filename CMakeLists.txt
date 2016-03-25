cmake_minimum_required(VERSION 2.8.3)
project(ihmc_ros_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  effort_controllers
  )

set(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(LIBRARIES ${PROJECT_NAME})

find_package(JNI REQUIRED)

include_directories(include ${JNI_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_subdirectory(csrc)

install(FILES ${PROJECT_SOURCE_DIR}/whole_robot_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY csrc/jvmLauncher/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/jvmLauncher
  FILES_MATCHING PATTERN "*.h"
  )

install(DIRECTORY csrc/ros_controllers/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros_controllers
  FILES_MATCHING PATTERN "*.h"
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  )
