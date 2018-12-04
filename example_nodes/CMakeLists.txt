cmake_minimum_required(VERSION 2.8.3)
project(example_nodes)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O2")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  urdf
  message_generation
  genmsg
  actionlib_msgs
  actionlib
  geometry_msgs
  cv_bridge
)

find_package(
  OpenCV REQUIRED
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TargetWaypoints.msg
  State.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GripperSrv.srv
  VisionSrv.srv
  CalibrateSrv.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ArmMotion.action
  BaseMotion.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp
  CATKIN_DEPENDS actionlib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/hebi
  include/hebi/Eigen
  include/util
  include/gains
  include/librealsense2
  include/librealsense2/examples

)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/example_nodes.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/example_nodes_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/free_roam.launch
  launch/trajectory_free_roam.launch
  launch/odometry.launch
  launch/arm_node.launch
  launch/demo1.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_example_nodes.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(Rosie_arm_node src/Rosie_Demo/Rosie_arm_node.cpp
  include/hebi/command.cpp
  include/hebi/feedback.cpp
  include/hebi/group_command.cpp
  include/hebi/group_feedback.cpp
  include/hebi/group_info.cpp
  include/hebi/group.cpp
  include/hebi/info.cpp
  include/hebi/robot_model.cpp
  include/hebi/log_file.cpp
  include/hebi/lookup.cpp
  include/hebi/mac_address.cpp
  include/hebi/trajectory.cpp

  include/util/grav_comp.hpp

  src/Rosie_Demo/arm_kinematics.cpp
  src/Rosie_Demo/arm_trajectory.cpp
  src/Rosie_Demo/arm.cpp
)
add_dependencies(Rosie_arm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(Rosie_arm_node ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/linux_x86_64/libhebi.so)

add_executable(Rosie_base_node src/Rosie_Demo/Rosie_base_node.cpp
  include/hebi/command.cpp
  include/hebi/feedback.cpp
  include/hebi/group_command.cpp
  include/hebi/group_feedback.cpp
  include/hebi/group_info.cpp
  include/hebi/group.cpp
  include/hebi/info.cpp
  include/hebi/robot_model.cpp
  include/hebi/log_file.cpp
  include/hebi/lookup.cpp
  include/hebi/mac_address.cpp
  include/hebi/trajectory.cpp

  include/util/grav_comp.hpp
)
add_dependencies(Rosie_base_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(Rosie_base_node ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/linux_x86_64/libhebi.so)

add_executable(demo_central src/Rosie_Demo/demo_central.cpp
  include/hebi/command.cpp
  include/hebi/feedback.cpp
  include/hebi/group_command.cpp
  include/hebi/group_feedback.cpp
  include/hebi/group_info.cpp
  include/hebi/group.cpp
  include/hebi/info.cpp
  include/hebi/robot_model.cpp
  include/hebi/log_file.cpp
  include/hebi/lookup.cpp
  include/hebi/mac_address.cpp
  include/hebi/trajectory.cpp
)
add_dependencies(demo_central ${catkin_EXPORTED_TARGETS})
target_link_libraries(demo_central ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/linux_x86_64/libhebi.so)


add_executable(gripper_node src/Rosie_Demo/gripper_node.cpp
  include/hebi/command.cpp
  include/hebi/feedback.cpp
  include/hebi/group_command.cpp
  include/hebi/group_feedback.cpp
  include/hebi/group_info.cpp
  include/hebi/group.cpp
  include/hebi/info.cpp
  include/hebi/robot_model.cpp
  include/hebi/log_file.cpp
  include/hebi/lookup.cpp
  include/hebi/mac_address.cpp
  include/hebi/trajectory.cpp
)
add_dependencies(gripper_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_node ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/linux_x86_64/libhebi.so)


add_executable(vision_process src/Rosie_Demo/vision_process.cpp)
add_dependencies(vision_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_process ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vision_threshold src/Rosie_Demo/vision_threshold.cpp)
add_dependencies(vision_threshold ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_threshold ${catkin_LIBRARIES} ${OpenCV_LIBS})






#add_executable(vision_process src/Rosie_Demo/vision_process.cpp
#  include/librealsense2/rs.hpp
#  include/librealsense2/examples/example.hpp
#)
#add_dependencies(vision_process ${catkin_EXPORTED_TARGETS})
#target_link_libraries(vision_process ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/linux_x86_64/libhebi.so)




#file (COPY ${PROJECT_SOURCE_DIR}/src/omnibase/omnibase_gains.xml
#      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/src/omnibase/)

#file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/include/gains/6-DoF-Arm-Gains-Rosie.xml
#      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/src/Rosie_Demo)

