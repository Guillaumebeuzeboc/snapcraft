cmake_minimum_required(VERSION 2.8.3)
project(ros-node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ros-node src/ros_node.cpp)

add_dependencies(ros-node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros-node ${catkin_LIBRARIES})

install(TARGETS ros-node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES node.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
