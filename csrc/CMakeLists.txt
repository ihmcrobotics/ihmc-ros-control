add_subdirectory(jvmLauncher)

include_directories(jvmLauncher)
add_executable(demo demo.cpp)

target_link_libraries(demo jvmLauncher)
