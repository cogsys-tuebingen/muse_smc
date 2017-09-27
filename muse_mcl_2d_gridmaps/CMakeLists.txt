cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_gridmaps)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  muse_mcl_2d_laser
  cslibs_indexed_storage
  nav_msgs
  pluginlib
  roscpp
  rostest
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_static_maps ${PROJECT_NAME}_providers ${PROJECT_NAME}_models
  CATKIN_DEPENDS muse_mcl_2d
)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_static_maps SHARED
    src/static_maps/binary_gridmap.cpp
    src/static_maps/distance_gridmap.cpp
    src/static_maps/probability_gridmap.cpp
    src/static_maps/likelihood_field_gridmap.cpp
)
target_link_libraries(${PROJECT_NAME}_static_maps
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_dynamic_maps SHARED
    src/dynamic_maps/probability_gridmap.cpp
)
target_link_libraries(${PROJECT_NAME}_static_maps
    ${catkin_LIBRARIES}
)


add_library(${PROJECT_NAME}_providers SHARED
    src/providers/binary_gridmap_provider.cpp
    src/providers/distance_gridmap_provider.cpp
    src/providers/probability_gridmap_provider.cpp
    src/providers/likelihood_field_gridmap_provider.cpp
    src/providers/binary_gridmap_service_provider.cpp
    src/providers/distance_gridmap_service_provider.cpp
    src/providers/probability_gridmap_service_provider.cpp
    src/providers/likelihood_field_gridmap_service_provider.cpp
)

target_link_libraries(${PROJECT_NAME}_providers
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_static_maps
)

add_library(${PROJECT_NAME}_models SHARED
    src/models/beam_model.cpp
    src/models/likelihood_field_model.cpp
    src/models/beam_model_amcl.cpp
    src/models/likelihood_field_model_amcl.cpp
    src/models/likelihood_field_prob_model_amcl.cpp
    src/models/beam_model_mle.cpp
    src/models/beam_model_parameter_estimator.cpp

    src/models/beam_model_log.cpp
    src/models/likelihood_field_model_log.cpp
    src/models/beam_model_amcl_normalized.cpp
    src/models/likelihood_field_model_amcl_normalized.cpp
    src/models/likelihood_field_model_pc.cpp
    src/models/likelihood_field_model_pc_log.cpp
)

target_link_libraries(${PROJECT_NAME}_models
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_static_maps
)

set(MUSE_MCL_2D_GRIDMAPS_PERF_TESTS True)

if(${MUSE_MCL_2D_GRIDMAPS_PERF_TESTS})
    add_executable(${PROJECT_NAME}_perf_runtimes_exp
        test/perf_runtimes_exp.cpp
    )

    add_executable(${PROJECT_NAME}_perf_runtimes_get_range
        test/perf_runtimes_get_range.cpp
    )

    target_link_libraries(${PROJECT_NAME}_perf_runtimes_get_range
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_static_maps
        ${OpenCV_LIBRARIES}
    )
    add_executable(${PROJECT_NAME}_perf_runtimes_line_iterator
        test/perf_runtimes_line_iterator.cpp
    )

    target_link_libraries(${PROJECT_NAME}_perf_runtimes_line_iterator
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_static_maps
        ${OpenCV_LIBRARIES}
    )
    add_executable(${PROJECT_NAME}_perf_runtimes_distance_transform
        test/perf_runtimes_distance_transform.cpp
    )

    target_link_libraries(${PROJECT_NAME}_perf_runtimes_distance_transform
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )
endif()

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_providers ${PROJECT_NAME}_models
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
