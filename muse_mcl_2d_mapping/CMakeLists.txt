cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d_gridmaps
  muse_mcl_2d_ndt
  muse_mcl_2d_laser
  nav_msgs
  muse_smc
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES muse_mcl_2d_mapping
#  CATKIN_DEPENDS muse_mcl_2d_gridmaps muse_mcl_2d_ndt
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_ocm_node
    src/occupancy_grid_mapper.cpp
    src/mapper.cpp
    src/occupancy_grid_mapper_node.cpp
)

target_link_libraries(${PROJECT_NAME}_ocm_node
    ${catkin_LIBRARIES}
)
