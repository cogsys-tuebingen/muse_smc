cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl)

## Additional cmake modules and configurations
## set c++11 compiler options active
include(cmake/c++11.cmake)
## arithmetic compiler flags
# include(cmake/muse_amcl-extras.cmake)
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

    add_unit_test(${PROJECT_NAME}_test_particle_set
        src/tests/test_particle_set.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test(${PROJECT_NAME}_test_distribution
        src/tests/test_distribution.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test(${PROJECT_NAME}_test_random
        src/tests/test_random.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test(${PROJECT_NAME}_test_angle
        src/tests/test_angle.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test(${PROJECT_NAME}_test_covariance
        src/tests/test_covariance.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test(${PROJECT_NAME}_test_bounding_rectangle
        src/tests/test_bounding_rectangle.cpp
        ${PROJECT_NAME}_particle_filter
    )


    ## optional tests that evaluate runtimes and such which
    ## do not have to fulfil unit tests
    add_executable(${PROJECT_NAME}_eval_particle_set_access_times
        src/examples/eval_particle_set_access_times.cpp
    )
    target_link_libraries(${PROJECT_NAME}_eval_particle_set_access_times
        muse_amcl_particle_filter
    )

    add_executable(${PROJECT_NAME}_eval_angle_normalization_times
        src/examples/eval_angle_normalization.cpp
    )

endif()

if(${${PROJECT_NAME}_BUILD_VISUAL_EXAMPLES})
    find_package(OpenCV QUIET)
    if(${OpenCV_FOUND})
        include_directories(
            ${OpenCV_INCLUDE_DIRS}
        )

        add_executable(${PROJECT_NAME}_exam_visualise_distributions
            src/examples/exam_visualise_distributions.cpp
        )
        target_link_libraries(muse_amcl_exam_visualise_distributions
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
        )

    else()
        message("Cannot build visual examples, OpenCV is missing!")
    endif()

    add_executable(${PROJECT_NAME}_exam_bounding_boxes
        src/examples/exam_bounding_boxes.cpp
    )
    target_link_libraries(${PROJECT_NAME}_exam_bounding_boxes
        ${catkin_LIBRARIES}
    )

endif()


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})





