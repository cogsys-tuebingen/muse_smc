cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cslibs_boost_geometry
  cslibs_indexed_storage
  cslibs_vectormaps
  muse_mcl
  nav_msgs
  pcl_ros
  pluginlib
  roscpp
  rostest
  sensor_msgs
)

catkin_package()


#add_library(${PROJECT_NAME}_particle_filter
#    src/particle_filter.cpp
#)

#add_dependencies(${PROJECT_NAME}_particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(${PROJECT_NAME}_particle_filter
#    ${catkin_LIBRARIES}
#    ${tinyXML_LIBRARIES}
#)


#add_executable(${PROJECT_NAME}_node
#    src/muse_mcl_node.cpp
#)

#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES}
#    ${PROJECT_NAME}_particle_filter
#)
