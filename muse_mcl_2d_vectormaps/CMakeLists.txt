cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_vectormaps)

find_package(catkin REQUIRED COMPONENTS
  cslibs_boost_geometry
  cslibs_vectormaps
  muse_smc
  pluginlib
  roscpp
  rostest
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/providers/provider_vectormap.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
