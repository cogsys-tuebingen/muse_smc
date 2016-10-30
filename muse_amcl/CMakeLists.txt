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
    LIBRARIES muse_amcl_particle_filter muse_amcl_plugins
    CFG_EXTRAS muse_amcl-extras.cmake
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp *.h)
add_custom_target(muse_show_header SOURCES ${ALL_HEADERS})

add_library(muse_amcl_tf
    src/tf/tf_provider.cpp
)
target_link_libraries(muse_amcl_tf
    ${catkin_LIBRARIES}
)

add_library(muse_amcl_plugins
    src/factories/update_function_factory.cpp
    src/factories/propagation_function_factory.cpp
    src/factories/map_provider_factory.cpp
    src/factories/data_provider_factory.cpp
)

target_link_libraries(muse_amcl_plugins
    ${catkin_LIBRARIES}
    ${tinyXML_LIBRARIES}
)


add_library(muse_amcl_particle_filter
    src/particle_filter/particle_filter.cpp
)

target_link_libraries(muse_amcl_particle_filter
    ${catkin_LIBRARIES}
    ${tinyXML_LIBRARIES}
)

if(${BUILD_EXAMPLES})
    add_executable(muse_amcl_exam_signals
        src/examples/example_signals.cpp
    )

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





