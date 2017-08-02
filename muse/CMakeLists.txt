cmake_minimum_required(VERSION 2.8.3)
project(muse)

include(cmake/muse_enable_c++11.cmake)
include(cmake/muse_extras.cmake)
set(OpenMP true)
include(cmake/muse_openmp.cmake)
include(cmake/muse_show_headers.cmake)
include(cmake/muse_add_unit_test_gtest.cmake)
include(cmake/muse_add_unit_test_rostest.cmake)
set(CATKIN_ENABLE_TESTING ON)
set(CATKIN_SKIP_TESTING OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pluginlib
)

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       muse_particle_filter
    CFG_EXTRAS      muse_extras.cmake
                    muse_add_unit_test_gtest.cmake
                    muse_enable_c++11.cmake
                    muse_show_headers.cmake
)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable (test_compile
    src/test_compile.cpp
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

