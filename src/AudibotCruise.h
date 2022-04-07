// ROS header
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <ugv_course_libs/gps_conv.h>
#include <tf/tf.h> 
#include <tf/transform_broadcaster.h> 
#include <geometry_msgs/Twist.h> 
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#define DEBUG 0

// Namespace matches ROS package name
namespace audibot_final_project{

class final_project
{
  public:
  final_project(ros::NodeHandle n, ros::NodeHandle pn);

  private:
  void algoTimerCallback(const ros::TimerEvent& event);
  void recvFix_a1(const sensor_msgs::NavSatFixConstPtr& msg);
  void recvFix_a2(const sensor_msgs::NavSatFixConstPtr& msg);
  void recvPathVel_a1(const geometry_msgs::TwistConstPtr& msg);
  void recvPathVel_a2(const geometry_msgs::TwistConstPtr& msg);
  void recvLaserScan_a1(const sensor_msgs::LaserScanConstPtr& msg);
  void recvCameraImage_a1(const sensor_msgs::ImageConstPtr& msg);
  void Level_1(void);
  void Level_2(void);
  void Level_3(void);
  void Level_4(void);
  ros::Subscriber gps_a1_sub;
  ros::Subscriber gps_a2_sub;
  ros::Subscriber vel_path_a1;
  ros::Subscriber vel_path_a2;
  ros::Subscriber laser_a1_sub;
  ros::Subscriber camera_a1_sub;
  ros::Publisher vel_a1_pub;
  ros::Publisher vel_a2_pub;
  ros::Publisher cloud_a1_pub;
  ros::Timer algoTimer;
};

}

