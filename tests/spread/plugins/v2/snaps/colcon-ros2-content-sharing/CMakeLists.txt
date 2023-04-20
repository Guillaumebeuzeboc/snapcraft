cmake_minimum_required(VERSION 3.5)
project(colcon_ros2_content_sharing)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package( Boost REQUIRED COMPONENTS
  filesystem
)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

include_directories(${Boost_INCLUDE_DIRS})

add_executable(colcon_ros2_content_sharing src/hello.cpp)
target_link_libraries(colcon_ros2_content_sharing ${Boost_LIBRARIES})
ament_target_dependencies(colcon_ros2_content_sharing rclcpp class_loader)

install(TARGETS
  colcon_ros2_content_sharing
  DESTINATION lib/${PROJECT_NAME})

ament_package()
