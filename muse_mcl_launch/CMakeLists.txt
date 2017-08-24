cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_launch)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  muse_mcl_2d_laser
  muse_mcl_2d_gridmaps
  muse_mcl_2d_odometry
)

catkin_package(
)
