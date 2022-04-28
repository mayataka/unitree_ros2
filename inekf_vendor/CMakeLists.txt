cmake_minimum_required(VERSION 3.11)
project(inekf_vendor)

# Default to C++17 to avoid alignment errors with -march=native
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(OPTIMIZE_FOR_NATIVE "Enable -march=native" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)

set(inekf_SRCDIR ${CMAKE_CURRENT_BINARY_DIR}/inekf_src)
set(inekf_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/inekf_install)

include(FetchContent)
FetchContent_GetProperties(inekf)
if(NOT inekf_POPULATED)
  FetchContent_Populate(
    inekf
    GIT_REPOSITORY https://github.com/mayataka/inekf
    GIT_TAG master
    GIT_SHALLOW TRUE
    SOURCE_DIR ${inekf_SRCDIR})
endif()

include(ExternalProject)
ExternalProject_Add(
  inekf
  SOURCE_DIR ${inekf_SRCDIR}
  BUILD_IN_SOURCE TRUE
  INSTALL_DIR ${inekf_DESTDIR}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} -DBUILD_TYPE=${CMAKE_BUILD_TYPE} -DOPTIMIZE_FOR_NATIVE=${OPTIMIZE_FOR_NATIVE} -DBUILD_PYTHONE_INTERFACE=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON 
)

ament_export_include_directories(include)
ament_export_libraries(inekf)
ament_export_dependencies(inekf pinocchio)

install(
  DIRECTORY ${inekf_DESTDIR}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

ament_package()