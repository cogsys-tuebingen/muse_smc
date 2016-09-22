cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdtree
  sensor_msgs
  tf
)

## OPENMP
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

## FLAGS ADITIONALE
## Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register)
add_definitions("-Ofast -march=native -ffast-math")
#add_definitions(-g -rdynamic)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES muse_amcl
#  CATKIN_DEPENDS geometry_msgs kdtree sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  src/
)

add_executable(muse_amcl_node
    src/node/muse_amcl_node.cpp

    src/tf/tf_provider.cpp

    src/maps/gridmaps/gridmap.cpp
    src/maps/gridmaps/binary_gridmap.cpp
    src/maps/gridmaps/probability_gridmap.cpp
    src/maps/gridmaps/distance_gridmap.cpp

    src/sensor/sensor.cpp

    src/odometry/odometry.cpp
)

target_link_libraries(muse_amcl_node
    ${catkin_LIBRARIES}
)

