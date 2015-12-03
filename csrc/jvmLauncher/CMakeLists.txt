add_library(jvmLauncher launcher.cpp)
target_link_libraries(jvmLauncher ${JNI_LIBRARIES})

install(TARGETS jvmLauncher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
