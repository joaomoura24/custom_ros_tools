cmake_minimum_required(VERSION 3.0.2)
project(custom_srvs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  SetTransform.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()
