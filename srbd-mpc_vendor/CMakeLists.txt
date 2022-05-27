cmake_minimum_required(VERSION 3.11)
project(srbd-mpc_vendor)

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

set(srbd-mpc_SRCDIR ${CMAKE_CURRENT_BINARY_DIR}/srbd-mpc_src)
set(srbd-mpc_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/srbd-mpc_install)

include(FetchContent)
FetchContent_GetProperties(srbd-mpc)
if(NOT srbd-mpc_POPULATED)
  FetchContent_Populate(
    srbd-mpc
    GIT_REPOSITORY https://github.com/mayataka/srbd-mpc
    GIT_TAG main
    GIT_SHALLOW TRUE
    SOURCE_DIR ${srbd-mpc_SRCDIR})
endif()

include(ExternalProject)
ExternalProject_Add(
  srbd-mpc
  SOURCE_DIR ${srbd-mpc_SRCDIR}
  BUILD_IN_SOURCE TRUE
  INSTALL_DIR ${srbd-mpc_DESTDIR}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)


ament_export_include_directories(include)
ament_export_libraries(srbd-mpc)
ament_export_dependencies(srbd-mpc)

install(
  DIRECTORY ${srbd-mpc_DESTDIR}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

ament_package()


