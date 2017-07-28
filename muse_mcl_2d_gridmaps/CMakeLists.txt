cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_gridmaps)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  muse_mcl
  nav_msgs
  pluginlib
  roscpp
  rostest
  tf
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES muse_mcl_2d_gridmaps
#  CATKIN_DEPENDS muse_mcl nav_msgs pluginlib roscpp rostest
#  DEPENDS system_lib
)

include_directories(
  include
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

add_library(${PROJECT_NAME} SHARED
    src/providers/provider_gridmap_binary.cpp
    src/providers/provider_gridmap_distance.cpp
    src/providers/provider_gridmap_probability.cpp
    src/providers/provider_gridmap_binary_service.cpp
    src/providers/provider_gridmap_distance_service.cpp
    src/providers/provider_gridmap_probability_service.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_maps
)


install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
