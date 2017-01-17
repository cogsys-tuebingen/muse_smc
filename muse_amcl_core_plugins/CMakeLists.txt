cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl_core_plugins)

set(CATKIN_ENABLE_TESTING ON)
set(CATKIN_SKIP_TESTING OFF)

find_package(catkin REQUIRED COMPONENTS
  muse_amcl
  nav_msgs
  pluginlib
  roscpp
  rostest
  sensor_msgs
  rostest
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/laser2D/beam_model.cpp
    src/laser2D/likelihood_field_model.cpp
    src/laser2D/likelihood_field_prob_model.cpp

    src/odometry/differential_drive.cpp
    src/odometry/omni_drive.cpp

    src/sampling/normal_2d.cpp
    src/sampling/uniform_all_maps_2d.cpp
    src/sampling/uniform_primary_map_2d.cpp

    src/resampling/wof.cpp
    src/resampling/multinomial.cpp
    src/resampling/residual.cpp
    src/resampling/stratified.cpp
    src/resampling/systematic.cpp
    src/resampling/kld_2d.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_library(muse_amcl_maps
    src/maps/binary_gridmap.cpp
    src/maps/distance_gridmap.cpp
    src/maps/probability_gridmap.cpp
)

target_link_libraries(muse_amcl_maps
    ${catkin_LIBRARIES}
)


if(${CATKIN_ENABLE_TESTING})
    add_custom_target(show_test_headers
        SOURCES
        test/test_map.hpp
        test/test_map_provider.hpp
        test/test_normal_2d.hpp
        test/test_uniform_primary_map_2d.hpp
        test/test_uniform_all_maps_2d.hpp
    )

    add_unit_test_rostest(${PROJECT_NAME}_test_node_sampling_normal_2d
        test/sampling_normal_2d.test
        test/test_node_sampling_normal_2d.cpp
        "${PROJECT_NAME}"
    )

    add_unit_test_rostest(${PROJECT_NAME}_test_node_sampling_uniform_all_maps_2d
        test/sampling_uniform_all_maps_2d.test
        test/test_node_sampling_uniform_all_maps_2d.cpp
       "${PROJECT_NAME}"
    )

    add_unit_test_rostest(${PROJECT_NAME}_test_node_sampling_uniform_primary_map_2d
        test/sampling_uniform_primary_map_2d.test
        test/test_node_sampling_uniform_primary_map_2d.cpp
       "${PROJECT_NAME}"
    )
endif()

if(True)
    add_library(muse_amcl_mock_plugins
        src/mock/mock_update.cpp
        src/mock/mock_data_provider.cpp
        src/mock/mock_propagation.cpp
        src/mock/mock_map_provider.cpp
    )

    target_link_libraries(muse_amcl_mock_plugins
        ${catkin_LIBRARIES}
    )

    add_executable(muse_amcl_exam_test_mock
        src/examples/exam_test_mock.cpp
    )
    target_link_libraries(muse_amcl_exam_test_mock
        ${catkin_LIBRARIES}
        ${tinyXML_LIBRARIES}
    )

    add_executable(muse_amcl_exam_test_mock_launch
        src/examples/exam_test_mock_launch.cpp
    )
    target_link_libraries(muse_amcl_exam_test_mock_launch
        ${catkin_LIBRARIES}
        ${tinyXML_LIBRARIES}
    )

    add_executable(muse_amcl_exam_test_mock_provider
        src/examples/exam_test_mock_provider.cpp
    )
    target_link_libraries(muse_amcl_exam_test_mock_provider
        ${catkin_LIBRARIES}
        ${tinyXML_LIBRARIES}
    )

    add_executable(muse_amcl_exam_resampling
        src/examples/exam_resampling.cpp
    )

    target_link_libraries(muse_amcl_exam_resampling
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
    )

    add_executable(muse_amcl_exam_vector
        src/examples/exam_vector.cpp
    )

    find_package(OpenCV QUIET)
    include_directories(
        ${OpenCV_INCLUDE_DIRS}
    )
    if(${OpenCV_FOUND})

        add_executable(muse_amcl_exam_map_line_iterator
            src/examples/exam_map_line_iterator.cpp
        )

        target_link_libraries(muse_amcl_exam_map_line_iterator
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
        )

        add_executable(muse_amcl_exam_distance_transoform
            src/examples/exam_distance_transform.cpp
        )
        target_link_libraries(muse_amcl_exam_distance_transoform
            ${OpenCV_LIBRARIES}
        )
    endif()
endif()

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
