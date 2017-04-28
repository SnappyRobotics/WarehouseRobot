cmake_minimum_required(VERSION 2.8.3)
project(warehouse_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	rospy
  geometry_msgs
  nav_msgs
  std_msgs
  tf
)

#generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES warehouse_robot
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(odom_calculator src/odom_calculator.cpp)
target_link_libraries(odom_calculator ${catkin_LIBRARIES})


install(TARGETS odom_calculator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY xacro/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/xacro)
