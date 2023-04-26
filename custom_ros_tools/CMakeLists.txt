cmake_minimum_required(VERSION 3.0.2)
project(custom_ros_tools)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_srvs
  rospy
  tf2
  tf2_ros
  )

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/resetable_static_transform_publisher
  scripts/remap_joint_state_to_floatarray
  scripts/remap_joint_state_to_joint_state
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
