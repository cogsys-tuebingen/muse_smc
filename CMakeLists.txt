cmake_minimum_required(VERSION 2.8.3)
project(muse_smc)

include(cmake/muse_smc_enable_c++17.cmake)
include(cmake/muse_smc_extras.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cslibs_math
  cslibs_time
  cslibs_utility
)

catkin_package(
    INCLUDE_DIRS    include
    CATKIN_DEPENDS  cslibs_time cslibs_math cslibs_utility
    DEPENDS         TinyXML
    CFG_EXTRAS      muse_smc_extras.cmake
                    muse_smc_enable_c++17.cmake
)

# add_definitions("-DMUSE_SMC_USE_DOTTY")
# add_definitions("-DMUSE_SMC_LOG_STATE")

include_directories(
  include
  ${Eigen3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

