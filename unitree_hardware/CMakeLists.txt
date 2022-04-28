cmake_minimum_required(VERSION 3.5)
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
  ${lcm_INCLUDE_DIRS}
)
link_directories(${PROJECT_SOURCE_DIR}/lib)
# target_compile_options(
#   ${PROJECT_NAME}
#   PUBLIC 
#   "-pthread"
# )

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

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
ament_package()