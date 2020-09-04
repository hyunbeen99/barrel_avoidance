cmake_minimum_required(VERSION 3.0.2)
project(barrel_avoidance)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  ackermann_msgs
  geometry_msgs
)
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs)

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(${PROJECT_NAME}
#	src/static_avoidance.cpp
#)

add_executable(barrel_avoidance_vlp16 src/static_avoidance_jinuk.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(barrel_avoidance_vlp16 ${catkin_LIBRARIES})

