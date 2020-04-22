cmake_minimum_required(VERSION 2.8.3)
project(muse_smc)

include(cmake/muse_smc_enable_c++17.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(TinyXML REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cslibs_math
  cslibs_time
  cslibs_utility
)

# add_definitions("-DMUSE_SMC_USE_DOTTY")
# add_definitions("-DMUSE_SMC_LOG_STATE")

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
)

include_directories(
  include
  ${Eigen3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

