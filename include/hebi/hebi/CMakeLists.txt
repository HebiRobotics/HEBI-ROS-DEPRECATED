cmake_minimum_required(VERSION 2.8)
project(hebi_cpp_api)

if(NOT DEFINED ARCH)
  set(ARCH "native")
endif()

include(cmake/hebi_platform.cmake)

if(HEBI_HOST_LINUX)
  set(HEBI_C_LIB_SUBDIR "lib/linux_${HEBI_TARGET_ARCH}")
elseif(HEBI_HOST_WINDOWS)
  set(HEBI_C_LIB_SUBDIR "lib/win_${HEBI_TARGET_ARCH}")
elseif(HEBI_HOST_OSX)
  set(HEBI_C_LIB_SUBDIR "lib/osx_${HEBI_TARGET_ARCH}")
else()
  message(FATAL_ERROR "Unkown host platform")
endif()

################
# Setup project

set(HEBI_CPP_INCLUDE_DIRECTORIES
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Eigen
  PARENT_SCOPE
)

set(HEBI_CPP_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/command.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/feedback.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/group_command.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/group.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/group_feedback.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/group_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lookup.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mac_address.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/log_file.cpp
  PARENT_SCOPE
)

set(HEBI_CPP_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/${HEBI_C_LIB_SUBDIR} PARENT_SCOPE)
set(HEBI_CPP_LIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${HEBI_C_LIB_SUBDIR} PARENT_SCOPE)