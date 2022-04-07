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

double lower_h = 100;
double lower_s = 150;
double lower_v = 0;

double upper_h = 140;
double upper_s = 255;
double upper_v = 255;

double largest_contour_area = 0;
double min_target_area = 1500;
double max_target_area = 2000;

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
    cv::namedWindow("Test_L", cv::WINDOW_NORMAL);
    cv::namedWindow("Test_R", cv::WINDOW_NORMAL);
  #endif
    // cv::namedWindow("Test_Project", cv::WINDOW_NORMAL);
    // cv::namedWindow("Blue", cv::WINDOW_NORMAL);
    // cv::namedWindow("Dilated", cv::WINDOW_NORMAL);
    // cv::namedWindow("Eroded", cv::WINDOW_NORMAL);
    // cv::namedWindow("Contours", cv::WINDOW_NORMAL);
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
    cv::Mat raw_img, mask_hsv, blue_hsv, erode_hsv, dilate_hsv;
    cv::Mat mask_img_left, mask_img_right, mask_img_proj;
    cv_bridge::CvImagePtr cv_ptr;

    std::vector<std::vector<cv::Point>> contours;
    cv::RNG rng(12345);



    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    raw_img = cv_ptr->image;

    int raw_img_height = raw_img.size().height;
    int raw_img_width = raw_img.size().width;

    // ROS_INFO("Image Rows: %d", raw_img.rows);
    // ROS_INFO("Image Cols: %d", raw_img.cols);
    // ROS_INFO("Image Size_H: %d", raw_img_height);
    // ROS_INFO("Image Size_W: %d", raw_img_width);


    // cv::Rect left_img_mask_roi(raw_img_width/2, 0, raw_img_width/2, raw_img_height/2);
    // mask_img_left = raw_img(left_img_mask_roi);

    // cv::Rect right_img_mask_roi(0, 0, raw_img_width/2, raw_img_height/2);
    // mask_img_right = raw_img(right_img_mask_roi);

    cv::Rect top_img_mask_roi(raw_img_width/5, 0, raw_img_width/2, 300);
    mask_img_proj = raw_img(top_img_mask_roi);

    

    cv::cvtColor(mask_img_proj, mask_hsv, CV_BGR2HSV);
    cv::inRange(mask_hsv, cv::Scalar(lower_h, lower_s, lower_v), cv::Scalar(upper_h, upper_s, upper_v), blue_hsv);

    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(-1, -1));
    cv::dilate(blue_hsv, dilate_hsv, dilate_kernel, cv::Point(-1,-1), 1);

    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1), cv::Point(-1, -1));
    cv::erode(dilate_hsv, erode_hsv, erode_kernel, cv::Point(-1,-1), 1);

    cv::findContours(erode_hsv, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    ROS_INFO("Contours Size: % d", contours.size());


    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f> centers(contours.size());
    std::vector<float> radius(contours.size());

    for(int i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(contours_poly[i]);
        cv::minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
        largest_contour_area = boundRect[0].area();
    }

    cv::Mat contours_img = cv::Mat::zeros(blue_hsv.size(), CV_8UC3);

    for(int i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
        cv::drawContours(contours_img, contours_poly, i, color);
        cv::rectangle(contours_img, boundRect[i].tl(), boundRect[i].br(), color, 2);
        cv::circle(contours_img, centers[i], (int)radius[i], color, 2);
    }
   


    //  cv::imshow("Test_Project", mask_img_proj);
    //  cv::imshow("Blue", blue_hsv);
    //  cv::imshow("Dilated", dilate_hsv); 
    //  cv::imshow("Eroded", erode_hsv);
    //  cv::imshow("Contours", contours_img);
     cv::waitKey(1);

  #if DEBUG
    cv::imshow("Raw", raw_img);
    cv::imshow("Blue", blue_hsv);
    cv::imshow("Test_L", mask_img_left);
    cv::imshow("Test_R", mask_img_right);
    cv::waitKey(2);
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

    /* TODO: Implement PID controller for linear.x to prevent collision */
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
    /* Values of a2 will be same as audibot_path_following package */
    cmd_vel_a2.linear.x = a2_path_linx;
    cmd_vel_a2.angular.z = a2_path_angz;

    /* TODO: Implement PID controller for linear.x to prevent collision */
    
    ROS_INFO("Largest Contour Area: %f", largest_contour_area);

    if (largest_contour_area > min_target_area && largest_contour_area < max_target_area)
    {
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
