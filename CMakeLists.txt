cmake_minimum_required(VERSION 2.8.3)
project(audibot_final_project)

add_message_files(
  FILES
  TrackedObject.msg
  TrackedObjectArray.msg
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
  cv_bridge
  pcl_ros
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(audibot_final_project
 src/audibot_cruise_node.cpp
 src/AudibotCruise.cpp
)

target_link_libraries(audibot_final_project
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
