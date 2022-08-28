cmake_minimum_required(VERSION 3.11)
project(legged_state_estimator_vendor)

# Default to C++17 to avoid alignment errors with -march=native
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(legged_state_estimator_SRCDIR ${CMAKE_CURRENT_BINARY_DIR}/legged_state_estimator_src)
set(legged_state_estimator_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/legged_state_estimator_install)

include(FetchContent)
FetchContent_GetProperties(legged_state_estimator)
if(NOT legged_state_estimator_POPULATED)
  FetchContent_Populate(
    legged_state_estimator
    GIT_REPOSITORY https://github.com/mayataka/legged_state_estimator
    GIT_TAG main
    GIT_SHALLOW TRUE
    SOURCE_DIR ${legged_state_estimator_SRCDIR})
endif()

include(ExternalProject)
ExternalProject_Add(
  legged_state_estimator
  SOURCE_DIR ${legged_state_estimator_SRCDIR}
  BUILD_IN_SOURCE TRUE
  INSTALL_DIR ${legged_state_estimator_DESTDIR}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)

ament_export_include_directories(include)
ament_export_libraries(legged_state_estimator)
ament_export_dependencies(legged_state_estimator pinocchio)

install(
  DIRECTORY ${legged_state_estimator_DESTDIR}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

ament_package()