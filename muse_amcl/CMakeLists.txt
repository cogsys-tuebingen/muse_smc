cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl)

## Additional cmake modules and configurations
## set c++11 compiler options active
include(cmake/c++11.cmake)
## arithmetic compiler flags
include(cmake/muse_amcl-extras.cmake)
## get openmp for parallel processing
set(OpenMP true)
include(cmake/open_mp.cmake)
## show all header files included in this project
include(cmake/show_headers.cmake)
## macros for unit tests
include(cmake/add_unit_test.cmake)

## add cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## get required dependencies
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
)

## activate tests while building
set(${PROJECT_NAME}_BUILD_TESTS True)
set(${PROJECT_NAME}_BUILD_VISUAL_EXAMPLES True)

## the catkin package definition
catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       muse_amcl_particle_filter
    CFG_EXTRAS      muse_amcl-extras.cmake c++11.cmake add_unit_test.cmake
)

## directories to be included for mandatory sources
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_tf
    src/tf/tf_provider.cpp
)
target_link_libraries(${PROJECT_NAME}_tf
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_particle_filter
    src/particle_filter/particle_filter.cpp
)

target_link_libraries(${PROJECT_NAME}_particle_filter
    ${catkin_LIBRARIES}
    ${tinyXML_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
    src/node/muse_amcl_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)


if(${${PROJECT_NAME}_BUILD_TESTS})
    include_directories(${GTEST_INCLUDE_DIRS})

    add_unit_test(${PROJECT_NAME}_test_signals
        src/tests/test_signals.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_executable(muse_amcl_exam_signals
        src/examples/exam_signals.cpp
    )

    add_executable(muse_amcl_exam_bounding_boxes
        src/examples/exam_bounding_boxes.cpp
    )
    target_link_libraries(muse_amcl_exam_bounding_boxes
        ${catkin_LIBRARIES}
    )

endif()

if(${${PROJECT_NAME}_BUILD_VISUAL_EXAMPLES})
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
            ${OpenCV_LIBRARIES}
        )

        add_executable(muse_amcl_exam_particle_set
            src/examples/exam_particle_set.cpp
        )
        target_link_libraries(muse_amcl_exam_particle_set
            muse_amcl_particle_filter
        )

    else()
        message("Cannot build visual examples, OpenCV is missing!")
    endif()

endif()

#if(${CSLIBS_BOOST_GEOMETRY_BUILD_TESTS})
#    include_directories(${GTEST_INCLUDE_DIRS})

#    add_unit_test(${PROJECT_NAME}_test_intersection
#                  src/tests/test_intersection.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_nearest_intersection
#                  src/tests/test_nearest_intersection.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_nearest_intersection_batch
#                  src/tests/test_nearest_intersection_batch.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_translation
#                  src/tests/test_translation.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_polar_line_set
#                  src/tests/test_polar_line_set.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_polygons
#                  src/tests/test_polygons.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_nearest_intersection_dist
#                  src/tests/test_nearest_intersection_dist.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_nearest_intersection_dist_batch
#                  src/tests/test_nearest_intersection_dist_batch.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_distance
#                  src/tests/test_distance.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_equality
#                  src/tests/test_equality.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_rotation
#                  src/tests/test_rotation.cpp
#                  ${PROJECT_NAME})

#    add_unit_test(${PROJECT_NAME}_test_line_angles
#                  src/tests/test_line_angles.cpp
#                  ${PROJECT_NAME})

#endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})





