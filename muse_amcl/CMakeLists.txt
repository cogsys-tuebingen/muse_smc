cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
find_package(TinyXML REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf
    roscpp
    visualization_msgs
    pluginlib
    cslibs_kdtree
)

set(BUILD_EXAMPLES True)

## Additional cmake modules and configurations
include(cmake/muse_amcl-extras.cmake)
include(cmake/open_mp.cmake)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES muse_amcl_particle_filter
    CFG_EXTRAS muse_amcl-extras.cmake
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    include/
    src/
)

file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)

add_library(muse_amcl_tf
    src/tf/tf_eigen.hpp
    src/tf/tf_map.hpp
    src/tf/tf_provider.cpp
)
target_link_libraries(muse_amcl_tf
    ${catkin_LIBRARIES}
)

add_library(muse_amcl_particle_filter
    ${ALL_HEADERS}

    src/particle_filter/resampling.hpp
    src/particle_filter/pose_generation.hpp
    src/particle_filter/particle_filter.cpp
    src/functions/update_function_factory.cpp
    src/functions/propagation_function_factory.cpp
)

target_link_libraries(muse_amcl_particle_filter
    ${catkin_LIBRARIES}
    ${tinyXML_LIBRARIES}
)

if(${BUILD_EXAMPLES})
    find_package(OpenCV QUIET)
    include_directories(
        ${OpenCV_INCLUDE_DIRS}
    )
    if(${OpenCV_FOUND})
        add_executable(muse_amcl_exam_visualise_distributions
            src/examples/exam_visualise_distributions.cpp
        )
        target_link_libraries(muse_amcl_exam_visualise_distributions
            ${OpenCV_LIBRARIES}
        )
        add_executable(muse_amcl_exam_particle_set
            src/examples/exam_particle_set.cpp
        )
        target_link_libraries(muse_amcl_exam_particle_set
            muse_amcl_particle_filter
        )
    endif()
endif()





