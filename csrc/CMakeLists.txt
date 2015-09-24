add_subdirectory(jvmLauncher)

include_directories(jvmLauncher)


# Add include files to QTCreator
FILE(GLOB_RECURSE INCLUDE_FILES "../include/*.h")
add_executable(demo demo.cpp ${INCLUDE_FILES})

target_link_libraries(demo jvmLauncher)
