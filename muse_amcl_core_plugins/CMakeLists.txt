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

include_directories(
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

if(${BUILD_EXAMPLES})
    add_executable(muse_amcl_exam_test_mock
        src/examples/exam_test_mock.cpp
    )
    target_link_libraries(muse_amcl_exam_test_mock
        muse_amcl_particle_filter
        ${catkin_LIBRARIES}
        ${tinyXML_LIBRARIES}
    )
    add_executable(muse_amcl_exam_test_mock_launch
        src/examples/exam_test_mock_launch.cpp
    )
    target_link_libraries(muse_amcl_exam_test_mock_launch
        muse_amcl_particle_filter
        ${catkin_LIBRARIES}
        ${tinyXML_LIBRARIES}
    )
endif()


install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
