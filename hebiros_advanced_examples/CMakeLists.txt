cmake_minimum_required(VERSION 2.8.3)
project(hebiros_advanced_examples)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O2")

find_package(catkin REQUIRED COMPONENTS
  hebiros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_set_gains_node src/other/example_set_gains_node.cpp)
add_executable(example_rviz_node src/rviz/example_rviz_node.cpp)
add_executable(example_moveit_node src/moveit/example_moveit_node.cpp)
add_executable(example_gazebo_node src/gazebo/example_gazebo_node.cpp)
add_executable(example_mobile_robot_node src/mobile_robot/example_mobile_robot_node.cpp)

add_dependencies(example_set_gains_node ${catkin_EXPORTED_TARGETS})
add_dependencies(example_rviz_node ${catkin_EXPORTED_TARGETS})
add_dependencies(example_moveit_node ${catkin_EXPORTED_TARGETS})
add_dependencies(example_gazebo_node ${catkin_EXPORTED_TARGETS})
add_dependencies(example_mobile_robot_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(example_set_gains_node ${catkin_LIBRARIES})
target_link_libraries(example_rviz_node ${catkin_LIBRARIES})
target_link_libraries(example_moveit_node ${catkin_LIBRARIES})
target_link_libraries(example_gazebo_node ${catkin_LIBRARIES})
target_link_libraries(example_mobile_robot_node ${catkin_LIBRARIES})


