add_library(jvmLauncher launcher.cpp)
target_link_libraries(jvmLauncher ${JNI_LIBRARIES})

set_property(TARGET jvmLauncher PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS jvmLauncher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
