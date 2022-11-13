cmake_minimum_required(VERSION 3.0.2)
project(mbf_gridmap_nav)

find_package(catkin REQUIRED COMPONENTS
  mbf_abstract_nav
  mbf_gridmap_core
  mbf_msgs
  std_srvs
  pluginlib
  roscpp
)

find_package(Boost COMPONENTS thread chrono REQUIRED)

set(MBF_GRIDMAP_SERVER_LIB mbf_gridmap_server)
set(MBF_GRIDMAP_SERVER_NODE mbf_gridmap_nav)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${MBF_GRIDMAP_SERVER_LIB}
  CATKIN_DEPENDS
    mbf_abstract_nav
    mbf_gridmap_core
    mbf_msgs
    std_srvs
    pluginlib
    roscpp
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${MBF_GRIDMAP_SERVER_LIB}
  src/gridmap_navigation_server.cpp
)
add_dependencies(${MBF_GRIDMAP_SERVER_LIB} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${MBF_GRIDMAP_SERVER_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${MBF_GRIDMAP_SERVER_NODE} src/move_base_server_node.cpp)
add_dependencies(${MBF_GRIDMAP_SERVER_NODE} ${MBF_GRIDMAP_SERVER_LIB})
target_link_libraries(${MBF_GRIDMAP_SERVER_NODE}
  ${MBF_GRIDMAP_SERVER_LIB}
  ${catkin_LIBRARIES}
)

install(TARGETS
  ${MBF_GRIDMAP_SERVER_LIB} ${MBF_GRIDMAP_SERVER_NODE}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  PATTERN ".svn" EXCLUDE
)

