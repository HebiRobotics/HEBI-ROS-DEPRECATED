cmake_minimum_required(VERSION 2.8.3)
project(hebiros_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  transmission_interface
)

catkin_package(
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  meshes
  urdf
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

