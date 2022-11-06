cmake_minimum_required(VERSION 3.0.2)
project(mbf_gridmap_core)

find_package(catkin REQUIRED COMPONENTS
  mbf_abstract_core
  mbf_utility
  grid_map_core
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    mbf_abstract_core
    mbf_utility
    grid_map_core
)

## Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
