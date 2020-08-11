cmake_minimum_required(VERSION 3.0.2)
project(muse_smc)

include(cmake/muse_smc_enable_c++17.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(TinyXML REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cslibs_math
  cslibs_time
  cslibs_utility
)

# Preferred compile flags:
#   -Wall -Werror     -> enable all warnings to be errors
#   -Ofast -ffastmath -> enables full performance and optimization

catkin_package(
    INCLUDE_DIRS
      include
    CATKIN_DEPENDS
      cslibs_time
      cslibs_math
      cslibs_utility
    DEPENDS
      TinyXML
    CFG_EXTRAS
      muse_smc_enable_c++17.cmake
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    include/
    ${catkin_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIR}
 )

add_library(${PROJECT_NAME}_build
  src/build.cpp
)

target_include_directories(${PROJECT_NAME}_build
  PRIVATE
    include/
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_build
  PRIVATE
    ${catkin_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

