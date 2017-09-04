cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_gridmaps)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  muse_mcl_2d_laser
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
  LIBRARIES ${PROJECT_NAME}_maps ${PROJECT_NAME}_providers ${PROJECT_NAME}_models
#  CATKIN_DEPENDS muse_mcl nav_msgs pluginlib roscpp rostest
#  DEPENDS system_lib
)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_maps SHARED
    src/maps/binary_gridmap.cpp
    src/maps/distance_gridmap.cpp
    src/maps/probability_gridmap.cpp
)
target_link_libraries(${PROJECT_NAME}_maps
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}_providers SHARED
    src/providers/binary_gridmap_provider.cpp
    src/providers/distance_gridmap_provider.cpp
    src/providers/probability_gridmap_provider.cpp
    src/providers/binary_gridmap_service_provider.cpp
    src/providers/distance_gridmap_service_provider.cpp
    src/providers/probability_gridmap_service_provider.cpp
)

target_link_libraries(${PROJECT_NAME}_providers
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_maps
)

add_library(${PROJECT_NAME}_models SHARED
    src/models/beam_model.cpp
    src/models/likelihood_field_model.cpp
    src/models/beam_model_amcl.cpp
    src/models/likelihood_field_model_amcl.cpp
    src/models/likelihood_field_prob_model_amcl.cpp
    src/models/beam_model_mle.cpp
    src/models/beam_model_parameter_estimator.cpp
)

target_link_libraries(${PROJECT_NAME}_models
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_maps
)

add_executable(${PROJECT_NAME}_example_distance_transform
    src/examples/distance_transform.cpp
)

target_link_libraries(${PROJECT_NAME}_example_distance_transform
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_runtimes_get_range
    src/examples/runtimes_get_range.cpp
)

target_link_libraries(${PROJECT_NAME}_runtimes_get_range
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_maps
)

add_executable(${PROJECT_NAME}_runtimes_line_iterator
    src/examples/runtimes_line_iterator.cpp
)

target_link_libraries(${PROJECT_NAME}_runtimes_line_iterator
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_maps
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_providers ${PROJECT_NAME}_models
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
