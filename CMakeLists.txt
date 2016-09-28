cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    cslibs_kdtree
    sensor_msgs
    tf
    )

set(BUILD_EXAMPLES True)

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
#ad_definitions("-Ofast -march=native -ffast-math")
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
    ${Eigen_INCLUDE_DIRS}
)

add_library(muse_amcl_maps
    src/maps/gridmap.hpp
    src/maps/binary_gridmap.cpp
    src/maps/distance_gridmap.cpp
    src/maps/probability_gridmap.cpp
    src/maps/distance_transform.hpp
)

target_link_libraries(muse_amcl_maps
    ${catkin_LIBRARIES}
)




add_custom_target(muse_acml_utility SOURCES
    src/math/distribution.hpp
    src/math/angle.hpp
    src/math/random.hpp
    src/math/math.hpp
)

add_library(muse_amcl_tf
    src/tf/tf_eigen.hpp
    src/tf/tf_map.hpp
    src/tf/tf_provider.cpp
)
target_link_libraries(muse_amcl_tf
    ${catkin_LIBRARIES}
)

add_library(muse_amcl_particle_filter
    src/particle_filter/update.hpp
    src/particle_filter/resampling.hpp
    src/particle_filter/propagation.hpp
    src/particle_filter/pose_generation.hpp
    src/particle_filter/particle_filter.cpp
)


add_executable(muse_amcl_node
    src/node/muse_amcl_node.cpp

    src/sensor/sensor.cpp

    src/odometry/odometry.cpp

    )

target_link_libraries(muse_amcl_node
    ${catkin_LIBRARIES}
    )


if(${BUILD_EXAMPLES})
    find_package(OpenCV QUIET)
    include_directories(
        ${OpenCV_INCLUDE_DIRS}
    )
    if(${OpenCV_FOUND})
        add_executable(muse_amcl_visualise_distributions
            src/examples/visualise_distributions.cpp
        )
        target_link_libraries(muse_amcl_visualise_distributions
            ${OpenCV_LIBRARIES}
        )
        add_executable(muse_amcl_distance_transoform
            src/examples/distance_transform.cpp
        )
        target_link_libraries(muse_amcl_distance_transoform
            ${OpenCV_LIBRARIES}
        )
    endif()
endif()





