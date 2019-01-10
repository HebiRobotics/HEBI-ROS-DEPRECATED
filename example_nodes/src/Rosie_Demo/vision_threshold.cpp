
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <chrono>
#include <thread>

#include <example_nodes/VisionSrv.h>
#include <example_nodes/CalibrateSrv.h>
#include <example_nodes/State.h>
#include <sensor_msgs/Image.h>

#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// Globals (TODO: refactor these...)
static const std::string OPENCV_WINDOW = "Image Thresholding GUI - Ctrl-P for Options";

sensor_msgs::Image input_image;

// Save image
void image_callback(sensor_msgs::Image data) {
  input_image = data;  
}

bool isHsv = false;
int lowR = 0;
int highR = 255;
int lowG = 0;
int highG = 255;
int lowB = 0;
int highB = 255;

void callbackButton(int button_id, void* userData) {
  isHsv = !isHsv;
}

void exportParameters(int button_id, void* userData) {
  if (isHsv)
    ROS_INFO_STREAM("{ " <<
      "hmin: " << lowR << ", hmax: " << highR << ", " <<
      "smin: " << lowG << ", smax: " << highG << ", " <<
      "vmin: " << lowB << ", vmax: " << highB << "}");
  else
    ROS_INFO_STREAM("{ " <<
      "rmin: " << lowR << ", rmax: " << highR << ", " <<
      "gmin: " << lowG << ", gmax: " << highG << ", " <<
      "bmin: " << lowB << ", bmax: " << highB << "}");
  ROS_WARN("Copy this into the parameters/color.txt file.  Note that you will need to set the LED color displayed on the modules -- the (r/g/b)disp variables -- and the priority variable as desired.");
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "vision_threshold");
  ros::NodeHandle node;

  // Get camera images
  ros::Subscriber image_subscriber = node.subscribe("/camera/color/image_raw", 60, image_callback);
      
  cv::namedWindow(OPENCV_WINDOW);
  cvCreateButton("Use HSV", callbackButton, nullptr, CV_CHECKBOX, 0);
  cvCreateButton("Export ROS Parameter", exportParameters, nullptr, CV_PUSH_BUTTON, 0);
//  cvCreateLabel("Red");
  cvCreateTrackbar("Low R/H", OPENCV_WINDOW.c_str(), &lowR, 255); // Red / Hue
  cvCreateTrackbar("High R/H", OPENCV_WINDOW.c_str(), &highR, 255); // Red / Hue
  cvCreateTrackbar("Low G/S", OPENCV_WINDOW.c_str(), &lowG, 255); // Green / Saturation
  cvCreateTrackbar("High G/S", OPENCV_WINDOW.c_str(), &highG, 255); // Green / Saturation
  cvCreateTrackbar("Low B/V", OPENCV_WINDOW.c_str(), &lowB, 255); // Blue / Value
  cvCreateTrackbar("High B/V", OPENCV_WINDOW.c_str(), &highB, 255); // Blue / Value

  cv::Mat img_thresh;
  cv::Mat img_hsv;

  while (ros::ok()) {

    cv_bridge::CvImagePtr cvImagePtr;
    size_t num_attempts = 5;
    size_t attempts_made = 0;
    for (attempts_made = 0; attempts_made < num_attempts; ++attempts_made) {
      try {
        // Try to convert the sensor image into a CV matrix
        cvImagePtr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);
        break;
      } catch (cv_bridge::Exception &e) {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        ROS_INFO("Failed to load stream. Trying again...");
      }
    }
    // Only continue and try to threshold image if valid frames
    // are being received.
    // (At end of for loop, iterator should equal max value...
    if (attempts_made != num_attempts) {
      if (isHsv) {
        cv::cvtColor(cvImagePtr->image, img_hsv, cv::COLOR_BGR2HSV); // convert from rgb to hsv
      } 
      cv::Mat* mat = isHsv ? &img_hsv : &cvImagePtr->image;

      // Threshold image based on slider values.
      cv::inRange(*mat, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), img_thresh);

      cv::imshow(OPENCV_WINDOW, img_thresh);
      cv::waitKey(2);

    }
    ros::spinOnce();
  }

  return 0;
}

