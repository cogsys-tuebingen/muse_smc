cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  visualization_msgs
  pluginlib
)

## Additional cmake modules and configurations
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/muse_amcl-extras.cmake)
include(cmake/open_mp.cmake)

## Find external libraries
find_package(Eigen3  REQUIRED)
find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CFG_EXTRAS muse_amcl-extras.cmake
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include
)

file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)

add_library(${PROJECT_NAME}
    # for visibility ...
    ${ALL_HEADERS}

    src/sensor/sensor_model.cpp
    src/sensor/sensor_model_factory.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${tinyXML_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
    src/muse_amcl_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)
