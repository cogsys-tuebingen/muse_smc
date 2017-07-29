cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_resampling)

find_package(catkin REQUIRED COMPONENTS
  cslibs_indexed_storage
  muse_mcl
  roscpp
  tf
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
    src/kld_2d.cpp
    src/systematic_2d.cpp
    src/residual_2d.cpp
    src/stratified_2d.cpp
    src/wheel_2d.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
