cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_stereo)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  cslibs_math_3d
  pluginlib
  roscpp
  rostest
  sensor_msgs
  tf
)
find_package(PCL REQUIRED COMPONENTS common filters segmentation io)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS muse_mcl_2d cslibs_math_3d
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
    src/providers/stereo_provider_2d.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
