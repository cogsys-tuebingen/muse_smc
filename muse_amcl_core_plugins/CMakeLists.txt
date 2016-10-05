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

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/mock/mock_update.cpp


)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
