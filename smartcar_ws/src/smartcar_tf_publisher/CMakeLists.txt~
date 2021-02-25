cmake_minimum_required(VERSION 2.8.3)
project(smartcar_tf_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pagv_tf_publisher
  CATKIN_DEPENDS geometry_msgs roscpp tf
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(smartcar_tf_publisher_node src/smartcar_tf_publisher_node.cpp)

 target_link_libraries(smartcar_tf_publisher_node
   ${catkin_LIBRARIES}
 )
