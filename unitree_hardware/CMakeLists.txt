cmake_minimum_required(VERSION 3.8)
project(unitree_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lcm REQUIRED)

# compile
add_library(${PROJECT_NAME} SHARED
  src/unitree_hardware.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)
target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  ${hardware_interface_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${rclcpp_LIBRARIES}
  lcm
  ${PROJECT_SOURCE_DIR}/lib/libunitree_legged_sdk_amd64.so
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${hardware_interface_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
link_directories(${PROJECT_SOURCE_DIR}/lib)

pluginlib_export_plugin_description_file(hardware_interface unitree_hardware.xml)

# install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

## exports
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()