cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_odometry)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl
  nav_msgs
  pluginlib
  roscpp
  tf
  cslibs_indexed_storage #<!-- REMOVE THIS -->
)

catkin_package(
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_providers
    src/providers/data_provider_odometry.cpp
    src/providers/data_provider_odometry_tf.cpp
)
target_link_libraries(${PROJECT_NAME}_providers
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_models
    src/models/differential_drive.cpp
    src/models/differential_drive_abs.cpp
    src/models/differential_drive_amcl.cpp
    src/models/omni_drive.cpp
)
target_link_libraries(${PROJECT_NAME}_models
    ${catkin_LIBRARIES}
)
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS ${PROJECT_NAME}_providers
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_models
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

