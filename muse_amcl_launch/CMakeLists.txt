cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl_launch)

find_package(catkin REQUIRED COMPONENTS
  muse_amcl
  muse_amcl_core_plugins
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES muse_amcl_launch
#  CATKIN_DEPENDS muse_amcl muse_amcl_core_plugins
#  DEPENDS system_lib
)
