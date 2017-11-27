cmake_minimum_required(VERSION 2.8.3)
project(hebiros_basic_examples)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O2")

find_package(catkin REQUIRED COMPONENTS
  hebiros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_01_lookup_node src/example_01_lookup_node.cpp)
add_executable(example_02_feedback_node src/example_02_feedback_node.cpp)
add_executable(example_03_command_node src/example_03_command_node.cpp)
add_executable(example_04_trajectory_node src/example_04_trajectory_node.cpp)

add_dependencies(example_01_lookup_node ${catkin_EXPORTED_TARGETS})
add_dependencies(example_02_feedback_node ${catkin_EXPORTED_TARGETS})
add_dependencies(example_03_command_node ${catkin_EXPORTED_TARGETS})
add_dependencies(example_04_trajectory_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(example_01_lookup_node ${catkin_LIBRARIES})
target_link_libraries(example_02_feedback_node ${catkin_LIBRARIES})
target_link_libraries(example_03_command_node ${catkin_LIBRARIES})
target_link_libraries(example_04_trajectory_node ${catkin_LIBRARIES})



