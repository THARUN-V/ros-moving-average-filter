cmake_minimum_required(VERSION 2.8.3)
project(sensor_filter_kit)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_filter_kit_lib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  sensor_filter_kit_lib src/sensor_filter_kit_lib.cpp
)

add_executable(${PROJECT_NAME}_node src/sensor_filter_kit_node.cpp)
add_executable(${PROJECT_NAME}_node_oop src/sensor_filter_kit_node_oop.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node_oop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME}_node_testing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  sensor_filter_kit_lib
  ${catkin_LIBRARIES}
  )

target_link_libraries(${PROJECT_NAME}_node_oop
  sensor_filter_kit_lib
  ${catkin_LIBRARIES}
  )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
