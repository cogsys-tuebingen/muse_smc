cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_laser)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  muse_mcl
  pluginlib
  roscpp
  rostest
  sensor_msgs
  tf
  muse_mcl_2d_gridmaps
  cslibs_indexed_storage # REMOVE THIS
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
    src/models/beam_model.cpp
    src/models/likelihood_field_model.cpp
    src/models/beam_model_amcl.cpp
    src/models/likelihood_field_model_amcl.cpp
    src/models/likelihood_field_prob_model_amcl.cpp
    src/models/beam_model_mle.cpp
    src/models/beam_model_parameter_estimator.cpp

    src/providers/provider_data_laser_2d.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
