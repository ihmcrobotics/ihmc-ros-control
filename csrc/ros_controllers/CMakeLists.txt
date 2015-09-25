add_library(ihmc_ros_control SHARED RRBotSimpleTestPositionController.cpp)
target_link_libraries(ihmc_ros_control ${catkin_LIBRARIES})

install(TARGETS
        ihmc_ros_control
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
