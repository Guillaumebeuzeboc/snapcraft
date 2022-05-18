cmake_minimum_required(VERSION 2.8.3)
project(ros-node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ros_node src/ros_node.cpp)

target_link_libraries(ros_node ${catkin_LIBRARIES})

install(TARGETS ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
