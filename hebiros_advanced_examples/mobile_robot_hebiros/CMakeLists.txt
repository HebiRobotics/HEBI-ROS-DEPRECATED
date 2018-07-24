cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot_hebiros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O2")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hebiros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omnibase
#  CATKIN_DEPENDS hebiros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(example_mobile_robot_node src/example_mobile_robot_node.cpp)

add_dependencies(example_mobile_robot_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(example_mobile_robot_node ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   src/key_read.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/mobile_robot.launch
  launch/mobile_robot_controller.launch
  launch/mobile_robot_planner.launch
  launch/mobile_robot_rtabmap.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


