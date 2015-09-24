cmake_minimum_required(VERSION 2.8.3)
project(ihmc_ros_control)

find_package(catkin REQUIRED)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})


include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(csrc)




