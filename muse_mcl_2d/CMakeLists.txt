cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d)

find_package(catkin REQUIRED COMPONENTS
  cslibs_indexed_storage
  nav_msgs
  pcl_ros
  pluginlib
  roscpp
  rostest
  sensor_msgs
  muse_smc
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS    include
)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_compilation
    src/compilation.cpp
)

target_link_libraries(${PROJECT_NAME}_compilation
    ${catkin_LIBRARIES}
)

#install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(TARGETS ${PROJECT_NAME}_providers ${PROJECT_NAME}_models
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
