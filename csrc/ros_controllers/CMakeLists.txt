add_library(ihmc_rrbot_controller SHARED rrbot_controllers/RRBotSimpleTestPositionController.cpp rrbot_controllers/SimplePDController.cpp)
target_link_libraries(ihmc_rrbot_controller ${catkin_LIBRARIES})

add_library(ihmc_valkyrie_controller SHARED ihmc_valkyrie_controllers/ValkyrieForwardCommandController.cpp rrbot_controllers/SimplePDController.cpp)
target_link_libraries(ihmc_valkyrie_controller ${catkin_LIBRARIES})

install(TARGETS
        ihmc_rrbot_controller
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
