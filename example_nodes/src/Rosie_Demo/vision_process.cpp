
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <example_nodes/TargetWaypoints.h>
#include <example_nodes/State.h>
#include <sensor_msgs/Image.h>

#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// Global Variables
static const std::string OPENCV_WINDOW = "Image window";
int rate_of_command = 60;

//inputs
sensor_msgs::Image input_image;
cv_bridge::CvImagePtr cvImagePtr;
sensor_msgs::Image depth_image;
cv_bridge::CvImagePtr dpImagePtr;
//middle
int height = 0;
int width = 0;


//outputs
geometry_msgs::Point arm_cmd;


void image_callback(sensor_msgs::Image data) {

  input_image = data;  
}


// void depth_callback(sensor_msgs::Image data) {
//   depth_image = data;
// }

geometry_msgs::Point transform_pixel2meters(int i, int j) {
  geometry_msgs::Point output;

  int start_i = 466;
  int start_j = 210;
  int pixel2meter = 280; // 280 pixels == 1 meter
  double dj, di;

  di = double(i - start_i) / pixel2meter;
  dj = double(j - start_j) / pixel2meter;
  ROS_INFO("%d %d %d %d", i, j, start_i, start_j);
  output.x = -dj;
  output.y = -di;
  output.z = -0.1;
  // ( -dj, -di, -0.1);
  ROS_INFO("Sending Rosie the following target: (%lg, %lg, %lg)", output.x, output.y, output.z);
  // output << -dj, -di;
  return output;
}



int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "vision_process");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  // INPUT
  ros::Subscriber image_subscriber = node.subscribe("/camera/color/image_raw", 60, image_callback);
  // ros::Subscriber depth_subscriber = node.subscribe("/camera/depth/image_rect_raw", 60, depth_callback);
  // ros::Subscriber depth_subscriber = node.subscribe("/camera/aligned_depth_to_color/image_raw", 60, depth_callback);

  // OUTPUT
  // ros::Publisher omni_publisher = node.advertise<geometry_msgs::Point>("/demo/cmd_blah", rate_of_command);
  ros::Publisher omni_publisher = node.advertise<geometry_msgs::Point>("/demo/target", rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  /******** MAIN LOOP **********/
  bool startup_complete1 = false;
  bool startup_complete2 = false;
  bool handoff_done = false;
  bool first_time = true;
  bool publish_red = false;
  // Eigen::VectorXd



  int red_x = 0;
  int red_y = 0;
  int red_num = 0;

  int blue_x = 0;
  int blue_y = 0;
  int blue_num = 0;

  int green_x = 0;
  int green_y = 0;
  int green_num = 0;

  while (ros::ok()) {
    cv::namedWindow(OPENCV_WINDOW);

    
    try {
      // Try to convert the sensor image into a CV matrix
      cvImagePtr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);
      startup_complete1 = true;
    } catch (cv_bridge::Exception &e) {
      // Avoid segmentation fault by not trying any later logic if emptry frames
      // are being received.
      startup_complete1 = false;
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // try {
    //   dpImagePtr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    //   startup_complete2 = true;
    // } catch (cv_bridge::Exception &e) {
    //   startup_complete2 = false;
    //   ROS_ERROR("Depth Exception: %s", e.what());
    // }


    if (startup_complete1) {
      cv::Mat &mat = cvImagePtr -> image;
      // cv::Mat &mat2 = dpImagePtr ->image;
      
      if (first_time) {

        /* 
        - Scan all the pixels on the page
        - Categorise them as red, blue, or yellow
        - Print and test a vector of x,y pixels for the three beanbags
        - Output the same pixels, but as meters ^_^ (will need physical set up)
        */

        height = mat.rows;
        width = mat.cols;


        // cv::Vec3b pixel; // = mat.at<cv::Vec3b>(i,j);

        ROS_INFO("(Width: %d) (Height: %d)", width, height);

        for (int i = 0; i < width; i++) {
          for (int j = 0; j < height; j++) {
            cv::Vec3b pixel = mat.at<cv::Vec3b>(j,i);
            // ROS_INFO("%d %d", i, j);
            // ROS_INFO("(%d. %d)", i, j);

            // std::cout << pixel << std::endl;
            
            /* The input format is BGR */
            // yellow
            if ((pixel[2] >= 200) && (pixel[1] > 200) && (pixel[0] < 100)){
              red_x += i;
              red_y += j;
              red_num += 1;
            }

            // blue 
            else if ((pixel[2] < 60) && (pixel[1] < 100) && (pixel[0] >= 140)){
              blue_x += i;
              // ROS_INFO("In here");
              blue_y += j;
              blue_num += 1;
            }
            else if ((pixel[2] < 160) && (pixel[1] >= 150) && (pixel[0] < 130 )){
              green_x += i;
              // ROS_INFO("In here");
              green_y += j;
              green_num += 1;
            }

          }
        }
        first_time = false;
      }

      

      int i = 0;
      int j = 0;

      if (red_num != 0) {
        i = red_x / red_num;
        j = red_y / red_num; 
        cv::circle(mat, cv::Point(i, j), 2, CV_RGB(0,255,255), 3);
        cv::circle(mat, cv::Point(i, j), 20, CV_RGB(0,255,255), 2);
        // ROS_INFO(Depth)

        if (!publish_red) {
          geometry_msgs::Point red_target = transform_pixel2meters(i,j);
          omni_publisher.publish(red_target); 
          publish_red = true;
        }

      }


      if (blue_num != 0) {
        i = blue_x / blue_num;
        j = blue_y / blue_num; 
        // cv::circle(mat, cv::Point(i, j), 2, CV_RGB(0,255,255), 3);
        // cv::circle(mat, cv::Point(i, j), 20, CV_RGB(0,255,255), 2);

      }

      // ROS_INFO("[%d, %d, %d]", green_x, green_y, green_num);

      if (green_num != 0) {
        i = green_x / green_num;
        j = green_y / green_num; 
        // cv::circle(mat, cv::Point(i, j), 2, CV_RGB(0,255,255), 3);
        // cv::circle(mat, cv::Point(i, j), 20, CV_RGB(0,255,255), 2);
        // ROS_INFO("Green is at (%d, %d)", i, j);

      }

      // ROS_INFO("(%d, %d)", i, j);

      // int i = 639;
      // int j = 479;
      // cv::Vec3b pixel = mat.at<cv::Vec3b>(i,j);

      cv::imshow(OPENCV_WINDOW, mat);

      cv::waitKey(3);


    }

    // int width = mat.cols;
    // int height = mat.rows;

    // ROS_INFO("WAZZAUP");

    ros::spinOnce();
    loop_rate.sleep();
    
  }
  return 0;
}

