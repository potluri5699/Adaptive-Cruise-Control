// Header file for the class
#include "AudibotCruise.h"

double a1_x = 0;
double a1_y = 0;
double a1_z = 0;
double a2_x = 0;
double a2_y = 0;
double a2_z = 0;

double a1_path_angz = 0;
double a1_path_linx = 0;
double a2_path_angz = 0;
double a2_path_linx = 0;

double dist_a1_a2 = 0;

const double target_dist_level_1 = 22;
const double target_dist_level_2 = 19;

const int laser_readings = 360;


geometry_msgs::Twist cmd_vel_a1, cmd_vel_a2;

laser_geometry::LaserProjection projector;
sensor_msgs::PointCloud2 cloud;

double laser_ranges[laser_readings];
double laser_angles[laser_readings];
double laser_max_distance = 0;
double laser_angle_at_max_distance = 0;
double lidar_distance = 0;

double lower_h = 110;
double lower_s = 150;
double lower_v = 150;

double upper_h = 110;
double upper_s = 150;
double upper_v = 150;

std::string param;
int level;

// Namespace matches ROS package name
namespace audibot_final_project {
  
  // Constructor with global and private node handle arguments
  final_project::final_project(ros::NodeHandle n, ros::NodeHandle pn)
  {
    gps_a1_sub =  n.subscribe("/a1/gps/fix", 1, &final_project::recvFix_a1, this);
    gps_a2_sub =  n.subscribe("/a2/gps/fix", 1, &final_project::recvFix_a2, this);
    laser_a1_sub = n.subscribe("/a1/front_laser/scan", 1, &final_project::recvLaserScan_a1, this);
    camera_a1_sub = n.subscribe("/a1/front_camera/image_raw", 1, &final_project::recvCameraImage_a1, this);
    vel_path_a1 =  n.subscribe("/a1/path/cmd_vel", 1, &final_project::recvPathVel_a1, this);
    vel_path_a2 =  n.subscribe("/a2/path/cmd_vel", 1, &final_project::recvPathVel_a2, this);

    vel_a1_pub = n.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1); 
    vel_a2_pub = n.advertise<geometry_msgs::Twist>("/a2/cmd_vel", 1);
    cloud_a1_pub = n.advertise<sensor_msgs::PointCloud2>("/a1/point_cloud", 1);

    algoTimer = n.createTimer(ros::Duration(0.01), &final_project::algoTimerCallback, this);


    pn.getParam("level", param);
    ROS_INFO("Entered Level: %s", param.c_str());

    if(param == "level1")
    {
      level = 1;
      ROS_INFO("Running Level 1");
    }
    else if(param == "level2")
    {
      level = 2;
      ROS_INFO("Running Level 2");
    }
    else if(param == "level3")
    {
      level = 3;
      ROS_INFO("Running Level 3");
    }
    else if(param == "pcl")
    {
      level = 4; /* Testing PCL data */
      ROS_INFO("Test PCL data");
    }
    else
    {
      level = 0;
    }
  #if DEBUG
    cv::namedWindow("Raw", cv::WINDOW_NORMAL);
    cv::namedWindow("Blue", cv::WINDOW_NORMAL);
  #endif
  }
  
  void final_project::recvFix_a1(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    UTMCoords current_coords_a1(*msg);

    a1_x = current_coords_a1.getX();
    a1_y = current_coords_a1.getY();
    a1_z = current_coords_a1.getZ();

    // ROS_INFO("a1 UTM Coodinates: (%f, %f, %f)", a1_x, a1_y, a1_z); 
  }

  void final_project::recvFix_a2(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    UTMCoords current_coords_a2(*msg);

    a2_x = current_coords_a2.getX();
    a2_y = current_coords_a2.getY();
    a2_z = current_coords_a2.getZ();

    // ROS_INFO("a2 UTM Coodinates: (%f, %f, %f)", a2_x, a2_y, a2_z);  
  }

  void final_project::recvPathVel_a1(const geometry_msgs::TwistConstPtr& msg)
  {
    a1_path_linx = msg->linear.x;
    a1_path_angz = msg->angular.z;
  }

  void final_project::recvPathVel_a2(const geometry_msgs::TwistConstPtr& msg)
  {
    a2_path_linx = msg->linear.x;
    a2_path_angz = msg->angular.z;
  }

  void final_project::recvLaserScan_a1(const sensor_msgs::LaserScanConstPtr& msg)
  {
    for(int i=0; i<laser_readings; i++){
      laser_ranges[i] = msg->ranges[i];
      laser_angles[i] = msg->angle_min + (i * msg->angle_increment);
    }

    projector.projectLaser(*msg, cloud);
    cloud_a1_pub.publish(cloud);
  }

  void final_project::recvCameraImage_a1(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat raw_hsv, img_mask;
    cv::cvtColor(raw_img, raw_hsv, CV_BGR2HSV);
    cv::inRange(raw_hsv, cv::Scalar(lower_h, lower_s, lower_v), cv::Scalar(upper_h, upper_s, upper_v), img_mask);

  #if DEBUG
    cv::imshow("Raw", raw_img);
    cv::imshow("Blue", img_mask);
    cv::waitKey(1);
  #endif

  }

  void final_project::algoTimerCallback(const ros::TimerEvent& event)
  {
    switch(level)
    {
      case 1:
        final_project::Level_1();
        break;
      case 2:
        final_project::Level_2();
        break;
      case 3:
        final_project::Level_3();
        break;
      case 4:
        final_project::Level_4();
        break;
      default:
        cmd_vel_a1.linear.x = 0;
        cmd_vel_a2.linear.x = 0;
        break;
    }
  }

  void final_project::Level_1(void)
  {
    /* Values of a2 will be same as audibot_path_following package */
    cmd_vel_a2.linear.x = a2_path_linx;
    cmd_vel_a2.angular.z = a2_path_angz;

    /* Implement PID controller for linear.x to prevent collision */
    dist_a1_a2 = sqrt(((a2_x - a1_x)*(a2_x - a1_x)) + ((a2_y - a1_y)*(a2_y - a1_y)) + ((a2_z - a1_z)*(a2_z - a1_z)));
    
    ROS_INFO("Controller Distance: %f", dist_a1_a2);

    if (dist_a1_a2 < target_dist_level_1){
      cmd_vel_a1.linear.x = 19;
      cmd_vel_a1.angular.z = a1_path_angz;
    }
    else{
      cmd_vel_a1.linear.x = a1_path_linx;
      cmd_vel_a1.angular.z = a1_path_angz;
      // ROS_INFO("a1 Linear Vel : %f", a1_path_linx);
      // ROS_INFO("a1 Angular Vel: %f", a1_path_angz);
    }

    vel_a1_pub.publish(cmd_vel_a1);
    vel_a2_pub.publish(cmd_vel_a2);

  }

  void final_project::Level_2(void)
  {
    for (int i=0; i<laser_readings; i++)
    {
      if(!(isinf(laser_ranges[i])))
      {
        if(laser_ranges[i] > laser_ranges[i-1]){
          laser_max_distance = laser_ranges[i];
          laser_angle_at_max_distance = laser_angles[i];
        }
      }
      else{
        /* Do Nothing */
        // ROS_INFO("Laser scan is not valid value i.e. infinity");
      }
    }

    double x = laser_max_distance * cos(laser_angle_at_max_distance);
    double y = laser_max_distance * sin(laser_angle_at_max_distance);
    lidar_distance = sqrt ((x*x) + (y*y));
    ROS_INFO("LIDAR Distance: %f", lidar_distance);

    // ROS_INFO("Laser scan at -45, 0, 45: (%f, %f, %f)", laser_ranges[0], laser_ranges[180], laser_ranges[359]);
    
    /* Speed control algorithm */
    cmd_vel_a2.linear.x = a2_path_linx;
    cmd_vel_a2.angular.z = a2_path_angz;

    if(lidar_distance < target_dist_level_2){
      cmd_vel_a1.linear.x = 19;
      cmd_vel_a1.angular.z = a1_path_angz;
    }
    else{
      cmd_vel_a1.linear.x = a1_path_linx;
      cmd_vel_a1.angular.z = a1_path_angz;
      // ROS_INFO("a1 Linear Vel : %f", a1_path_linx);
      // ROS_INFO("a1 Angular Vel: %f", a1_path_angz);
    }

    vel_a1_pub.publish(cmd_vel_a1);
    vel_a2_pub.publish(cmd_vel_a2);
  }

  void final_project::Level_3(void)
  {

  }

  void final_project::Level_4(void)
  {
    /* Values of a2 will be same as audibot_path_following package */
    cmd_vel_a2.linear.x = a2_path_linx;
    cmd_vel_a2.angular.z = a2_path_angz;
    
    /* Changed the a1 speed to match with a2 */
    cmd_vel_a1.linear.x = 20;
    cmd_vel_a1.angular.z = a1_path_angz;

    vel_a1_pub.publish(cmd_vel_a1);
    vel_a2_pub.publish(cmd_vel_a2);

  }
}
