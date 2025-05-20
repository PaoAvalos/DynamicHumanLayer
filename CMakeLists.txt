cmake_minimum_required(VERSION 3.0.2)
project(dynamic_human_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  pcl_ros
  people_msgs
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/DynamicHumanLayer.cfg
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cdynamic_human_layer
  CATKIN_DEPENDS roscpp costmap_2d pluginlib dynamic_reconfigure visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(${PROJECT_NAME} src/dynamic_human_layer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(dynamic_human_layer ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
