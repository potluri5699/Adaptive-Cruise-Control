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

const double target_dist = 22;

const int laser_readings = 360;


geometry_msgs::Twist cmd_vel_a1, cmd_vel_a2;

double laser_ranges[laser_readings];
double laser_angles[laser_readings];


// Namespace matches ROS package name
namespace audibot_final_project {
  
  // Constructor with global and private node handle arguments
  final_project::final_project(ros::NodeHandle n, ros::NodeHandle pn)
  {
    gps_a1_sub =  n.subscribe("/a1/gps/fix", 1, &final_project::recvFix_a1, this);
    gps_a2_sub =  n.subscribe("/a2/gps/fix", 1, &final_project::recvFix_a2, this);
    laser_a1_aub = n.subscribe("/a1/front_laser/scan", 1, &final_project::recvLaserScan_a1, this);
    vel_path_a1 =  n.subscribe("/a1/path/cmd_vel", 1, &final_project::recvPathVel_a1, this);
    vel_path_a2 =  n.subscribe("/a2/path/cmd_vel", 1, &final_project::recvPathVel_a2, this);

    vel_a1_pub = n.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1); 
    vel_a2_pub = n.advertise<geometry_msgs::Twist>("/a2/cmd_vel", 1); 

    level1_timer = n.createTimer(ros::Duration(0.01), &final_project::level1TimerCallback, this);
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
    // ROS_INFO("Laser scan at -45, 0, 45: (%f, %f, %f)", laser_ranges[0], laser_ranges[180], laser_ranges[359]);
  }

  void final_project::level1TimerCallback(const ros::TimerEvent& event)
  {
    /* Values of a2 will be same as audibot_path_following package */
    cmd_vel_a2.linear.x = a2_path_linx;
    cmd_vel_a2.angular.z = a2_path_angz;

    /* Implement PID controller for linear.x to prevent collision */
    dist_a1_a2 = sqrt(((a2_x - a1_x)*(a2_x - a1_x)) + ((a2_y - a1_y)*(a2_y - a1_y)) + ((a2_z - a1_z)*(a2_z - a1_z)));
    
    ROS_INFO("Distance: %f", dist_a1_a2);

    if (dist_a1_a2 < target_dist){
      cmd_vel_a1.linear.x = 19;
      cmd_vel_a1.angular.z = a1_path_angz;
    }
    else{
      cmd_vel_a1.linear.x = a1_path_linx;
      cmd_vel_a1.angular.z = a1_path_angz;
      ROS_INFO("a1 Linear Vel : %f", a1_path_linx);
      ROS_INFO("a1 Angular Vel: %f", a1_path_angz);
    }




    vel_a1_pub.publish(cmd_vel_a1);
    vel_a2_pub.publish(cmd_vel_a2);
  }

  
}
