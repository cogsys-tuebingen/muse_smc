cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl)

## Additional cmake modules and configurations
## set c++11 compiler options active
include(cmake/muse_amcl_enable_c++11.cmake)
## arithmetic compiler flags
# include(cmake/muse_amcl-extras.cmake)
## get openmp for parallel processing
set(OpenMP true)
include(cmake/muse_amcl_open_mp.cmake)
## show all header files included in this project
include(cmake/muse_amcl_show_headers.cmake)
## macros for unit tests
include(cmake/muse_amcl_add_unit_test_gtest.cmake)
include(cmake/muse_amcl_add_unit_test_rostest.cmake)
set(CATKIN_ENABLE_TESTING ON)
set(CATKIN_SKIP_TESTING OFF)

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
    cslibs_indexed_storage
    geometry_msgs
    message_generation
)

add_service_files(
   FILES
   GlobalInitialization.srv
   PoseInitialization.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

## the catkin package definition
catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       muse_amcl_particle_filter
    CFG_EXTRAS      muse_amcl_extras.cmake
                    muse_amcl_add_unit_test_gtest.cmake
                    muse_amcl_enable_c++11.cmake
                    muse_amcl_show_headers.cmake
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

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)



if(${CATKIN_ENABLE_TESTING})
    add_unit_test_gtest(${PROJECT_NAME}_test_signals
        test/test_signals.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_gtest(${PROJECT_NAME}_test_random
        test/test_random.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_gtest(${PROJECT_NAME}_test_pose_generators
        test/test_pose_generators.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_gtest(${PROJECT_NAME}_test_angle
        test/test_angle.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_gtest(${PROJECT_NAME}_test_covariance
        test/test_covariance.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_gtest(${PROJECT_NAME}_test_bounding_rectangle
        test/test_bounding_rectangle.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_gtest(${PROJECT_NAME}_test_index
        test/test_index.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_gtest(${PROJECT_NAME}_test_indexation
        test/test_indexation.cpp
        ${PROJECT_NAME}_particle_filter
    )

    add_unit_test_rostest(${PROJECT_NAME}_test_node_distribution
        test/test_distribution.test
        test/test_node_distribution.cpp
        "${catkin_LIBRARIES};yaml-cpp"
    )

    add_unit_test_rostest(${PROJECT_NAME}_test_node_clustering
        test/test_clustering.test
        test/test_node_clustering.cpp
        "${catkin_LIBRARIES};yaml-cpp"
    )

    add_unit_test_rostest(${PROJECT_NAME}_test_node_particle_set
        test/test_particle_set.test
        test/test_node_particle_set.cpp
        "${catkin_LIBRARIES};yaml-cpp"
    )

    add_unit_test_rostest(${PROJECT_NAME}_test_node_tf_provider
        test/test_tf_provider.test
        test/test_node_tf_provider.cpp
        "${catkin_LIBRARIES}"
    )

    add_executable(${PROJECT_NAME}_eval_particle_set_access_times
        src/examples/eval_particle_set_access_times.cpp
    )
    target_link_libraries(${PROJECT_NAME}_eval_particle_set_access_times
        muse_amcl_particle_filter
    )

    add_executable(${PROJECT_NAME}_eval_angle_normalization_times
        src/examples/eval_angle_normalization.cpp
    )


    add_executable(${PROJECT_NAME}_eval_functional
        src/examples/eval_functional.cpp
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





