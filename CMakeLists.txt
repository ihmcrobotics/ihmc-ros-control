cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")
project(ihmc_ros_control)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        joint_limits_interface
        transmission_interface
        realtime_tools
        controller_manager
        controller_interface
        hardware_interface
        controller_manager_tests
        controller_manager_msgs
        control_toolbox
        imu_sensor_controller
        force_torque_sensor_controller
        position_controllers
        velocity_controllers
        effort_controllers
        forward_command_controller
        joint_state_controller
        joint_trajectory_controller
        diff_drive_controller
        gripper_action_controller
        rqt_joint_trajectory_controller
)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

find_package(JNI REQUIRED)

include_directories(include ${JNI_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_subdirectory(csrc)




