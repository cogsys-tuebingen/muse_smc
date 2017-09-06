cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_laser)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  pluginlib
  roscpp
  rostest
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES muse_mcl_2d_laser
#  CATKIN_DEPENDS muse_mcl pluginlib roscpp rostest sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/providers/laser_provider_2d.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_iteration_runtimes
    src/test/test_runtimes.cpp
)
target_link_libraries(${PROJECT_NAME}_iteration_runtimes
    ${catkin_LIBRARIES}
)


install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
