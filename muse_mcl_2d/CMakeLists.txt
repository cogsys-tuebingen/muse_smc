cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d)

find_package(catkin REQUIRED COMPONENTS
  cslibs_indexed_storage
  nav_msgs
  sensor_msgs
  pcl_ros
  pluginlib
  roscpp
  rostest
  muse_smc
  visualization_msgs
  pluginlib
  geometry_msgs
  message_generation
  std_msgs
)

find_package(Eigen3 REQUIRED)

add_service_files(
   FILES
   GlobalInitialization.srv
   PoseInitialization.srv
)

add_message_files(
    FILES
    SampleMsg.msg
    SampleSetMsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${PROJECT_NAME}_resampling ${PROJECT_NAME}_sampling
    CATKIN_DEPENDS  muse_smc
    DEPENDS         Eigen3
)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_resampling
    src/resampling/kld_2d.cpp
    src/resampling/systematic_2d.cpp
    src/resampling/residual_2d.cpp
    src/resampling/stratified_2d.cpp
    src/resampling/wheel_2d.cpp
)

target_link_libraries(${PROJECT_NAME}_resampling
    ${catkin_LIBRARIES}
)


add_library(${PROJECT_NAME}_sampling
    src/sampling/normal_2d.cpp
    src/sampling/uniform_primary_map_2d.cpp
    src/sampling/uniform_all_maps_2d.cpp
)

target_link_libraries(${PROJECT_NAME}_sampling
    ${catkin_LIBRARIES}
)

add_definitions("-DMUSE_SMC_USE_DOTTY")
add_definitions("-DMUSE_SMC_LOG_STATE")

add_executable(${PROJECT_NAME}_node
    src/node/muse_mcl_2d_node.cpp
    src/node/state_publisher_2d.cpp
    src/node/sample_set_publisher_2d.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

add_unit_test_gtest(${PROJECT_NAME}_test_transform2d
    test/test_transform2d.cpp
    "${catkin_LIBRARIES}"
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_resampling ${PROJECT_NAME}_sampling
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
