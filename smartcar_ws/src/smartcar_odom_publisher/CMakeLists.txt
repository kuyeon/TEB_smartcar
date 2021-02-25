cmake_minimum_required(VERSION 2.8.3)
project(smartcar_odom_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pagv_odom_publisher
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

 add_executable(smartcar_odom_publisher_node src/smartcar_odom_publisher_node.cpp src/math_util.cpp)

 target_link_libraries(smartcar_odom_publisher_node
   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
 )

