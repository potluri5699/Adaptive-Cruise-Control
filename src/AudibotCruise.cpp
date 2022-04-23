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

const double target_dist_level_1_max = 22;
const double target_dist_level_1_min = 12;

const double target_dist_level_2_max = 19;
const double target_dist_level_2_min = 12;


const int laser_readings = 360;

std::string param;
geometry_msgs::Twist cmd_vel_a1, cmd_vel_a2;
laser_geometry::LaserProjection projector;
sensor_msgs::PointCloud2 cloud_msg, merged_cloud;
audibot_final_project::TrackedObjectArray bboxes, dummy_boxes;
std_msgs::UInt32  box_id_1, box_id_2;
std_msgs::Float64 box_x_1, box_x_2;
std_msgs::Float64 box_y_1, box_y_2;
std_msgs::Float64 box_z_1, box_z_2;

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
double target_min_contour_area = 1500;
double target_max_contour_area = 2100;

unsigned int a2TimerCount = 0;
int level;


// Namespace matches ROS package name
namespace audibot_final_project {
  
  // Constructor with global and private node handle arguments
  final_project::final_project(ros::NodeHandle n, ros::NodeHandle pn):
    kd_tree(new pcl::search::KdTree<pcl::PointXYZ>)
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
    merged_cloud_a1_pub = n.advertise<sensor_msgs::PointCloud2>("a1_merged_cloud", 1);
    bboxes_pub = n.advertise<audibot_final_project::TrackedObjectArray>("a2_object", 1);

    algoTimer = n.createTimer(ros::Duration(0.01), &final_project::algoTimerCallback, this);
    a2Timer = n.createTimer(ros::Duration(0.01), &final_project::a2TimerCallback, this);




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
      ROS_INFO("Testing PCL data");
    }
    else
    {
      level = 0;
    }
  #if DEBUG
    cv::namedWindow("Raw", cv::WINDOW_NORMAL);
    // cv::namedWindow("Test_L", cv::WINDOW_NORMAL);
    // cv::namedWindow("Test_R", cv::WINDOW_NORMAL);
    cv::namedWindow("Test_ROI", cv::WINDOW_NORMAL);
    cv::namedWindow("Blue", cv::WINDOW_NORMAL);
    cv::namedWindow("Dilated", cv::WINDOW_NORMAL);
    cv::namedWindow("Eroded", cv::WINDOW_NORMAL);
    cv::namedWindow("Contours", cv::WINDOW_NORMAL);
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

    projector.projectLaser(*msg, cloud_msg);
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
    // ROS_INFO("Contours Size: % d", contours.size());


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
   

  #if DEBUG
     cv::imshow("Raw", raw_img);
     cv::imshow("Project_ROI", mask_img_proj);
     cv::imshow("Blue", blue_hsv);
     cv::imshow("Dilated", dilate_hsv); 
     cv::imshow("Eroded", erode_hsv);
     cv::imshow("Contours", contours_img);
    // cv::imshow("Test_L", mask_img_left);
    // cv::imshow("Test_R", mask_img_right);
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
        final_project::PCL();
        break;
      default:
        cmd_vel_a1.linear.x = 0;
        cmd_vel_a2.linear.x = 0;
        break;
    }
  }

  void final_project::a2TimerCallback(const ros::TimerEvent& event)
  {
    if(level == 4)
    {
      /* Values of a2 will be same as audibot_path_following package */
      cmd_vel_a2.linear.x = a2_path_linx;
      cmd_vel_a2.angular.z = a2_path_angz;
    }
    else
    {
      /* Implement a timer counter to vary the a2 values*/  
      if(a2TimerCount >= 0 && a2TimerCount < 18000) /* 3 minutes */
      {
        if(a2TimerCount >= 0 && a2TimerCount < 3000) /* 30 seconds */
        {
          cmd_vel_a2.linear.x = 15;
          cmd_vel_a2.angular.z = a2_path_angz;
        }
        else if(a2TimerCount >= 3000 && a2TimerCount < 6000)  /* 60 seconds */
        {
          cmd_vel_a2.linear.x = 18;
          cmd_vel_a2.angular.z = a2_path_angz;    
        }
        else if(a2TimerCount >= 6000 && a2TimerCount < 10000)  /* 100 seconds */
        { 
          cmd_vel_a2.linear.x = 22;
          cmd_vel_a2.angular.z = a2_path_angz;
        }
        else if(a2TimerCount >= 10000 && a2TimerCount < 14000)  /* 140 seconds */
        {
          cmd_vel_a2.linear.x = 18;
          cmd_vel_a2.angular.z = a2_path_angz;  
        }
        else  /* 180 seconds */
        {
          cmd_vel_a2.linear.x = 21;
          cmd_vel_a2.angular.z = a2_path_angz;  
        }
#if 0
        if(a2TimerCount >= 0 && a2TimerCount < 4000)
        {
          cmd_vel_a2.linear.x = a2_path_linx;
          cmd_vel_a2.angular.z = a2_path_angz;
        }
        else if(a2TimerCount >= 4000 && a2TimerCount < 8000) 
        {
          cmd_vel_a2.linear.x = 24;
          cmd_vel_a2.angular.z = a2_path_angz;    
        }
        else if(a2TimerCount >= 8000 && a2TimerCount < 11000)  
        { 
          cmd_vel_a2.linear.x = 21;
          cmd_vel_a2.angular.z = a2_path_angz;
        }
        else if(a2TimerCount >= 11000 && a2TimerCount < 14000) 
        {
          cmd_vel_a2.linear.x = 18;
          cmd_vel_a2.angular.z = a2_path_angz;  
        }
        else  
        {
          cmd_vel_a2.linear.x = 16;
          cmd_vel_a2.angular.z = a2_path_angz;  
        }
#endif
      }
      else if(a2TimerCount >= 18000 && a2TimerCount < 36000) /* 6 minutes */
      {
        cmd_vel_a2.linear.x = 18;
        cmd_vel_a2.angular.z = a2_path_angz;
      }
      else if(a2TimerCount >= 36000 && a2TimerCount < 54000) /* 9 minutes */
      {
        cmd_vel_a2.linear.x = 16;
        cmd_vel_a2.angular.z = a2_path_angz;
      }
      else if(a2TimerCount >= 54000 && a2TimerCount < 78000) /* 13 minutes */
      {
        cmd_vel_a2.linear.x = 19;
        cmd_vel_a2.angular.z = a2_path_angz;
      }
      else if(a2TimerCount >= 78000 && a2TimerCount < 102000)  /* 17 minutes */
      {
        cmd_vel_a2.linear.x = 24;
        cmd_vel_a2.angular.z = a2_path_angz;
      }
      else if(a2TimerCount >= 102000 && a2TimerCount < 120000)  /* 20 minutes */
      {
        cmd_vel_a2.linear.x = 20;
        cmd_vel_a2.angular.z = a2_path_angz;  
        a2TimerCount = 0;
      }
      else /* Never */
      {
        cmd_vel_a2.linear.x = a2_path_linx;
        cmd_vel_a2.angular.z = a2_path_angz;
      }

      a2TimerCount++;
    }

    vel_a2_pub.publish(cmd_vel_a2);
  }

  void final_project::Level_1(void)
  {
    /* Values of a2 will be same as audibot_path_following package */
    // cmd_vel_a2.linear.x = a2_path_linx;
    // cmd_vel_a2.angular.z = a2_path_angz;

    /* TODO: Implement PID controller for linear.x to prevent collision */
    dist_a1_a2 = sqrt(((a2_x - a1_x)*(a2_x - a1_x)) + ((a2_y - a1_y)*(a2_y - a1_y)) + ((a2_z - a1_z)*(a2_z - a1_z)));
    
    ROS_INFO("Controller Distance: %f", dist_a1_a2);

    if (dist_a1_a2 < target_dist_level_1_max && dist_a1_a2 > target_dist_level_1_min){
      cmd_vel_a1.linear.x = 16;
      cmd_vel_a1.angular.z = a1_path_angz;
    }
    else if (dist_a1_a2 < target_dist_level_1_min)
    {
      cmd_vel_a1.linear.x = 0;
      cmd_vel_a1.angular.z = 0;
    }
    else
    {
      cmd_vel_a1.linear.x = a1_path_linx;
      cmd_vel_a1.angular.z = a1_path_angz;
      // ROS_INFO("a1 Linear Vel : %f", a1_path_linx);
      // ROS_INFO("a1 Angular Vel: %f", a1_path_angz);
    }

    vel_a1_pub.publish(cmd_vel_a1);
    // vel_a2_pub.publish(cmd_vel_a2);

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
    
    /* Values of a2 will be same as audibot_path_following package */
    // cmd_vel_a2.linear.x = a2_path_linx;
    // cmd_vel_a2.angular.z = a2_path_angz;

    if(lidar_distance < target_dist_level_2_max && lidar_distance > target_dist_level_2_min){
      cmd_vel_a1.linear.x = 16;
      cmd_vel_a1.angular.z = a1_path_angz;
    }
    else if(lidar_distance < target_dist_level_2_min)
    {
      cmd_vel_a1.linear.x = 0;
      cmd_vel_a1.angular.z = 0;
    }
    else
    {
      cmd_vel_a1.linear.x = a1_path_linx;
      cmd_vel_a1.angular.z = a1_path_angz;
      // ROS_INFO("a1 Linear Vel : %f", a1_path_linx);
      // ROS_INFO("a1 Angular Vel: %f", a1_path_angz);
    }

    vel_a1_pub.publish(cmd_vel_a1);
    // vel_a2_pub.publish(cmd_vel_a2);
  }

  void final_project::Level_3(void)
  {
    /* Values of a2 will be same as audibot_path_following package */
    // cmd_vel_a2.linear.x = a2_path_linx;
    // cmd_vel_a2.angular.z = a2_path_angz;

    /* TODO: Implement PID controller for linear.x to prevent collision */
    
    ROS_INFO("Largest Contour Area: %f", largest_contour_area);

    if (largest_contour_area > target_min_contour_area && largest_contour_area < target_max_contour_area)
    {
      cmd_vel_a1.linear.x = 16;
      cmd_vel_a1.angular.z = a1_path_angz;
    }
    else if(largest_contour_area > target_max_contour_area)
    {
      cmd_vel_a1.linear.x = 0;
      cmd_vel_a1.angular.z = 0;
    }
    else
    {
      cmd_vel_a1.linear.x = a1_path_linx;
      cmd_vel_a1.angular.z = a1_path_angz;
      // ROS_INFO("a1 Linear Vel : %f", a1_path_linx);
      // ROS_INFO("a1 Angular Vel: %f", a1_path_angz); 
    }

    vel_a1_pub.publish(cmd_vel_a1);
    // vel_a2_pub.publish(cmd_vel_a2);

  }

  void final_project::PCL(void)
  {
    /* Values of a2 will be same as audibot_path_following package */
    cmd_vel_a2.linear.x = a2_path_linx;
    cmd_vel_a2.angular.z = a2_path_angz;
    
    /* Changed the a1 speed to match with a2 */
    cmd_vel_a1.linear.x = 20;
    cmd_vel_a1.angular.z = a1_path_angz;

    vel_a1_pub.publish(cmd_vel_a1);
    vel_a2_pub.publish(cmd_vel_a2);

    cloud_a1_pub.publish(cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthroughFilter(input_cloud, filtered_cloud);
    voxelFilter(filtered_cloud, filtered_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    normalsFilter(filtered_cloud, no_ground_cloud);

    // Return from function here if there are no points left after filtering out the ground points
    if (no_ground_cloud->size() == 0)
    {
      dummy_boxes.header = pcl_conversions::fromPCL(no_ground_cloud->header);
      bboxes_pub.publish(dummy_boxes);
      return;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    euclideanClustering(no_ground_cloud, cluster_clouds);

    generateBoundingBoxes(cluster_clouds);
    bboxes.header = pcl_conversions::fromPCL(filtered_cloud->header);

    mergeClusters(cluster_clouds);
    merged_cloud.header = pcl_conversions::fromPCL(filtered_cloud->header);

    merged_cloud_a1_pub.publish(merged_cloud);
    bboxes_pub.publish(bboxes);
  }

  void final_project::passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    pcl::IndicesPtr roi_indices(new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud(cloud_in);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 100);
    pass.filter(*roi_indices);

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices(roi_indices);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 10);
    pass.filter(*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices(roi_indices);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2, 3);
    pass.filter(*cloud_out);
  
  }

  void final_project::voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud_out);
    downsample.setLeafSize(0.2, 0.2, 0.2);
    downsample.filter(*cloud_out);
  }

  void final_project::normalsFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Compute normal vectors for the incoming point cloud
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    kd_tree->setInputCloud(cloud_in);
    normal_estimator.setSearchMethod(kd_tree);
    normal_estimator.setInputCloud(cloud_in);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*cloud_normals);

    // Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;
    
      for (int i = 0; i < cloud_normals->points.size(); i++) {
      double mag_x = cloud_normals->points[i].normal_x * cloud_normals->points[i].normal_x;
      double mag_y = cloud_normals->points[i].normal_y * cloud_normals->points[i].normal_y;
      double mag_z = cloud_normals->points[i].normal_z * cloud_normals->points[i].normal_z;
       
      double vertical_angle = acos(cloud_normals->points[i].normal_z);

      if((vertical_angle > (30 * M_PI/180)) && (vertical_angle < (150 * M_PI/180)))
      {
        non_vertical_normals.indices.push_back(i);
      }

    }
    
    // Copy non-vertical normals into a separate cloud
    pcl::copyPointCloud(*cloud_in, non_vertical_normals, *cloud_out);

    for (int i = 0; i < non_vertical_normals.indices.size(); i++) {
      geometry_msgs::Pose p;
      p.position.x = cloud_in->points[non_vertical_normals.indices[i]].x;
      p.position.y = cloud_in->points[non_vertical_normals.indices[i]].y;
      p.position.z = cloud_in->points[non_vertical_normals.indices[i]].z;

      double nx = cloud_normals->points[non_vertical_normals.indices[i]].normal_x;
      double ny = cloud_normals->points[non_vertical_normals.indices[i]].normal_y;
      double nz = cloud_normals->points[non_vertical_normals.indices[i]].normal_z;

      // Construct rotation matrix to align frame transform with the normal vector
      tf2::Matrix3x3 rot_mat;
      // First basis vector is the vector we want to align
      rot_mat[0] = tf2::Vector3(nx, ny, nz);
      if (std::abs(nz) < 0.9) {
        // Vector is not close to vertical --> use x and y components to create orthogonal vector
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
      } else {
        // Vector is close to vertical --> use y and z components to make orthogonal vector
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
      }
      // Normalize the generated orthogonal vector, because it is not necessarily unit length
      rot_mat[1].normalize();
      // Cross product produces the third basis vector of the rotation matrix
      rot_mat[2] = rot_mat[0].cross(rot_mat[1]);

      tf2::Quaternion q;
      rot_mat.transpose().getRotation(q);

      // Fill orientation of pose structure
      tf2::convert(q, p.orientation);
    }
  }
  void final_project::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(1); /* ClusterTolerance distance between two closest cluster points */
    ec.setMinClusterSize(2);   /* ClusterSize is number of cluser points required to produce a bounding box */
    ec.setMaxClusterSize(5000);
    kd_tree->setInputCloud(cloud_in);
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    for (auto indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud_in, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster);
    }
  }
  
  void final_project::mergeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto& cluster : cluster_clouds) {
      merged_pcl_cloud->points.insert(merged_pcl_cloud->points.begin(), cluster->points.begin(), cluster->points.end());
    }
    merged_pcl_cloud->width = merged_pcl_cloud->points.size();
    merged_pcl_cloud->height = 1;
    merged_pcl_cloud->is_dense = true;

    pcl::toROSMsg(*merged_pcl_cloud, merged_cloud);  
  }

  void final_project::generateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    pcl::PointXYZ min_point, max_point;
    
    bboxes.objects.clear();
    int bbox_id = 0;

    for (auto& cluster : cluster_clouds) {
      pcl::getMinMax3D(*cluster, min_point, max_point);
      audibot_final_project::TrackedObject box;
      box.header = bboxes.header;

      box.spawn_time = ros::Time::now();

      box.bounding_box_scale.x = max_point.x - min_point.x;
      box.bounding_box_scale.y = max_point.y - min_point.y;
      box.bounding_box_scale.z = max_point.z - min_point.z;
      box.pose.position.x = 0.5 * (max_point.x + min_point.x);
      box.pose.position.y = 0.5 * (max_point.y + min_point.y);
      box.pose.position.z = 0.5 * (max_point.z + min_point.z);
      box.pose.orientation.w = 1.0;
      box.id = bbox_id++;
      bboxes.objects.push_back(box);
    }

    if(bboxes.objects.size() > 0)
    {
      box_id_1.data = bboxes.objects[0].id;
      box_x_1.data  = bboxes.objects[0].pose.position.x;
      box_y_1.data  = bboxes.objects[0].pose.position.y;
      box_z_1.data  = bboxes.objects[0].pose.position.z;
      
      ROS_INFO("PCL Distance_1: %d /n", box_x_1.data);

      box_id_2.data = bboxes.objects[1].id;
      box_x_2.data  = bboxes.objects[1].pose.position.x;
      box_y_2.data  = bboxes.objects[1].pose.position.y;
      box_z_2.data  = bboxes.objects[1].pose.position.z;

      ROS_INFO("PCL Distance_1: %d /n", box_x_2.data);
    }
  }
}