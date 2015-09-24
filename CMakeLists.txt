cmake_minimum_required(VERSION 2.6)
project(IHMCROSControl)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${CMAKE_SOURCE_DIR}/bin CACHE PATH "IHMCROSControl install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include)

add_subdirectory(csrc)
