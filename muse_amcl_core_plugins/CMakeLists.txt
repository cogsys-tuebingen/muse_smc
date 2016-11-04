cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl_core_plugins)


find_package(catkin REQUIRED COMPONENTS
  muse_amcl
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
)

catkin_package()

set(BUILD_EXAMPLES True)

###########
## Build ##
###########

file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp *.h)
add_custom_target(muse_core_show_header SOURCES ${ALL_HEADERS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/mock/mock_update.cpp
    src/mock/mock_propagation.cpp

    src/laser2D/beam_model.cpp
    src/laser2D/likelihood_field_model.cpp
    src/laser2D/likelihood_field_prob_model.cpp

    src/odometry/differential_drive.cpp
    src/odometry/omni_drive.cpp
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


if(${BUILD_EXAMPLES})
    add_library(muse_amcl_mock_plugins
        src/mock/mock_update.cpp
        src/mock/mock_data_provider.cpp
        src/mock/mock_propagation.cpp
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
