add_subdirectory(jvmLauncher)
add_subdirectory(ros_controllers)

add_executable(demo demo.cpp)
target_link_libraries(demo jvmLauncher)
