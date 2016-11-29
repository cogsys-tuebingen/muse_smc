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
include(cmake/show_headers.cmake)

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
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(muse_amcl_tf
    src/tf/tf_provider.cpp
)
target_link_libraries(muse_amcl_tf
    ${catkin_LIBRARIES}
)

add_library(muse_amcl_particle_filter
    src/particle_filter/particle_filter.cpp
)

target_link_libraries(muse_amcl_particle_filter
    ${catkin_LIBRARIES}
    ${tinyXML_LIBRARIES}
)

add_executable(muse_amcl_node
    src/node/muse_amcl_node.cpp
)

target_link_libraries(muse_amcl_node
    ${catkin_LIBRARIES}
)


if(${BUILD_EXAMPLES})
    add_executable(muse_amcl_exam_signals
        src/examples/exam_signals.cpp
    )

    add_executable(muse_amcl_exam_bounding_boxes
        src/examples/exam_bounding_boxes.cpp
    )
    target_link_libraries(muse_amcl_exam_bounding_boxes
        ${catkin_LIBRARIES}
    )

    find_package(OpenCV QUIET)
    if(${OpenCV_FOUND})
        include_directories(
            ${OpenCV_INCLUDE_DIRS}
        )
        add_executable(muse_amcl_exam_visualise_distributions
            src/examples/exam_visualise_distributions.cpp
        )
        target_link_libraries(muse_amcl_exam_visualise_distributions
            ${catkin_LIBRARIES}
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





