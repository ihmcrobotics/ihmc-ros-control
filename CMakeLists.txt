cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")
project(ihmc_ros_control)

find_package(catkin REQUIRED COMPONENTS
        controller_interface
        hardware_interface
        effort_controllers
)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

find_package(JNI REQUIRED)

include_directories(include ${JNI_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_subdirectory(csrc)




