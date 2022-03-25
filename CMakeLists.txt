cmake_minimum_required(VERSION 2.8.3)
project(audibot_final_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(audibot_final_project
 src/audibot_cruise_node.cpp
 src/AudibotCruise.cpp
)

target_link_libraries(audibot_final_project
  ${catkin_LIBRARIES}
)
