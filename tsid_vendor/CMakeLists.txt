cmake_minimum_required(VERSION 3.11)
project(tsid_vendor)

# Default to C++17 to avoid alignment errors with -march=native
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(OPTIMIZE_FOR_NATIVE "Enable -march=native" OFF)
set(CMAKE_CXX_ADDITIONAL_FLAGS "")
if (OPTIMIZE_FOR_NATIVE)
  set(CMAKE_CXX_ADDITIONAL_FLAGS "-march=native")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(tsid_SRCDIR ${CMAKE_CURRENT_BINARY_DIR}/tsid_src)
set(tsid_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/tsid_install)

include(FetchContent)
FetchContent_GetProperties(tsid)
if(NOT tsid_POPULATED)
  FetchContent_Populate(
    tsid
    GIT_REPOSITORY https://github.com/stack-of-tasks/tsid
    GIT_TAG master
    GIT_SHALLOW TRUE
    SOURCE_DIR ${tsid_SRCDIR})
endif()

include(ExternalProject)
ExternalProject_Add(
  tsid
  SOURCE_DIR ${tsid_SRCDIR}
  BUILD_IN_SOURCE TRUE
  INSTALL_DIR ${tsid_DESTDIR}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_CXX_FLAGS=${CMAKE_CXX_ADDITIONAL_FLAGS} -DBUILD_PYTHONE_INTERFACE=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)


ament_export_include_directories(include)
ament_export_libraries(tsid)
ament_export_dependencies(tsid pinocchio)

install(
  DIRECTORY ${tsid_DESTDIR}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

ament_package()