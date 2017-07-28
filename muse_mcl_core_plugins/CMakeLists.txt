cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_core_plugins)

set(CATKIN_ENABLE_TESTING ON)
set(CATKIN_SKIP_TESTING OFF)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl
  nav_msgs
  pluginlib
  roscpp
  rostest
  sensor_msgs
  rostest
  cslibs_indexed_storage
  pcl_ros
  muse_mcl_2d_gridmaps
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED

    src/resampling/wof.cpp
    src/resampling/multinomial.cpp
    src/resampling/residual.cpp
    src/resampling/stratified.cpp
    src/resampling/systematic.cpp
    src/resampling/kld_2d.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

#if(${CATKIN_ENABLE_TESTING})
#    add_custom_target(show_test_headers
#        SOURCES
#        test/test_map.hpp
#        test/test_map_provider.hpp
#        test/test_normal_2d.hpp
#        test/test_uniform_primary_map_2d.hpp
#        test/test_uniform_all_maps_2d.hpp
#    )

#    add_unit_test_rostest(${PROJECT_NAME}_test_node_sampling_normal_2d
#        test/sampling_normal_2d.test
#        test/test_node_sampling_normal_2d.cpp
#        "${PROJECT_NAME}"
#    )

#    add_unit_test_rostest(${PROJECT_NAME}_test_node_sampling_uniform_all_maps_2d
#        test/sampling_uniform_all_maps_2d.test
#        test/test_node_sampling_uniform_all_maps_2d.cpp
#       "${PROJECT_NAME}"
#    )

#    add_unit_test_rostest(${PROJECT_NAME}_test_node_sampling_uniform_primary_map_2d
#        test/sampling_uniform_primary_map_2d.test
#        test/test_node_sampling_uniform_primary_map_2d.cpp
#       "${PROJECT_NAME}"
#    )

#    add_unit_test_rostest(${PROJECT_NAME}_test_node_gridmap_providers
#        test/gridmap_providers.test
#        test/test_node_gridmap_providers.cpp
#        "${PROJECT_NAME}"
#    )

#endif()

#if(True)
#    add_executable(muse_mcl_exam_resampling
#        src/examples/exam_resampling.cpp
#    )

#    target_link_libraries(muse_mcl_exam_resampling
#        ${catkin_LIBRARIES}
#        ${PROJECT_NAME}
#    )

#    add_executable(muse_mcl_exam_vector
#        src/examples/exam_vector.cpp
#    )

#    add_executable(${PROJECT_NAME}_test_node_odometry
#        test/test_node_odometry.cpp
#    )
#    target_link_libraries(${PROJECT_NAME}_test_node_odometry
#        ${catkin_LIBRARIES}
#    )

#    find_package(OpenCV QUIET)
#    include_directories(
#        ${OpenCV_INCLUDE_DIRS}
#    )
#    if(${OpenCV_FOUND})

#        add_executable(muse_mcl_test_node_gridmap
#            test/test_node_gridmap.cpp
#        )

#        target_link_libraries(muse_mcl_test_node_gridmap
#            ${catkin_LIBRARIES}
#            ${OpenCV_LIBRARIES}
#            ${PROJECT_NAME}
#        )

#        add_executable(muse_mcl_test_node_gridmap_particles
#            test/test_node_gridmap_particles.cpp
#        )

#        target_link_libraries(muse_mcl_test_node_gridmap_particles
#            ${catkin_LIBRARIES}
#            ${OpenCV_LIBRARIES}
#            ${PROJECT_NAME}
#        )

#        add_executable(muse_mcl_exam_map_line_iterator
#            src/examples/exam_map_line_iterator.cpp
#        )

#        target_link_libraries(muse_mcl_exam_map_line_iterator
#            ${OpenCV_LIBRARIES}
#            ${catkin_LIBRARIES}
#        )
#    endif()
#endif()

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
