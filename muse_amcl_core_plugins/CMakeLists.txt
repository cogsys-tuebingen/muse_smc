cmake_minimum_required(VERSION 2.8.3)
project(muse_amcl_core_plugins)

find_package(catkin REQUIRED COMPONENTS
  muse_amcl
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/sensor_models/mock_sensor_model.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
