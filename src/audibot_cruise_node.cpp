// ROS and Class header file
#include <ros/ros.h>
#include "AudibotCruise.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "audibot_final_project");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  audibot_final_project::final_project node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
