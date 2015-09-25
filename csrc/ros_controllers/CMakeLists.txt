add_library(ihmc_rrbot_controller SHARED RRBotSimpleTestPositionController.cpp SimplePDController.cpp)
target_link_libraries(ihmc_rrbot_controller ${catkin_LIBRARIES})

install(TARGETS
        ihmc_rrbot_controller
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
