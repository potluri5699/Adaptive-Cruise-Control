// ROS header
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <ugv_course_libs/gps_conv.h>
#include <tf/tf.h> 
#include <tf/transform_broadcaster.h> 
#include <geometry_msgs/Twist.h> 
#include <math.h>

// Namespace matches ROS package name
namespace audibot_final_project{

class final_project
{
  public:
  final_project(ros::NodeHandle n, ros::NodeHandle pn); 

  private:
  void level1TimerCallback(const ros::TimerEvent& event);
  void recvFixa1(const sensor_msgs::NavSatFixConstPtr& msg);
  void recvFixa2(const sensor_msgs::NavSatFixConstPtr& msg);
  void recvPathVela1(const geometry_msgs::TwistConstPtr& msg);
  void recvPathVela2(const geometry_msgs::TwistConstPtr& msg);
  ros::Subscriber gps_a1_sub;
  ros::Subscriber gps_a2_sub;
  ros::Subscriber vel_path_a1;
  ros::Subscriber vel_path_a2;
  ros::Publisher vel_a1_pub;
  ros::Publisher vel_a2_pub;
  ros::Timer level1_timer;

};

}

