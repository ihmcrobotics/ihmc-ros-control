add_library(ihmc_rrbot_controller SHARED RRBotSimpleTestPositionController.cpp)
target_link_libraries(ihmc_rrbot_controller ${catkin_LIBRARIES})


add_library(ihmc_ros_control_java_bridge SHARED IHMCRosControlJavaBridge.cpp)
target_link_libraries(ihmc_ros_control_java_bridge ${catkin_LIBRARIES})

install(TARGETS
        ihmc_rrbot_controller ihmc_ros_control_java_bridge
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
