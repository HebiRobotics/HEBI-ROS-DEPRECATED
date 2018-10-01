
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
static const std::string OPENCV_WINDOW = "Image window";
sensor_msgs::Image input_image;

// TODO: add "register color", with color bounds; parameterize set/has/getcenter by color structure
class Blobs {
public:
  Blobs() = default;

  void setYellow(int x, int y, int num) {
    yellow_x = x;
    yellow_y = y;
    yellow_num = num;
  }
  void setBlue(int x, int y, int num) {
    blue_x = x;
    blue_y = y;
    blue_num = num;
  }
  void setGreen(int x, int y, int num) {
    green_x = x;
    green_y = y;
    green_num = num;
  }
  bool hasYellow() { return yellow_num != 0; }
  bool hasBlue() { return blue_num != 0; }
  bool hasGreen() { return green_num != 0; }
  cv::Point getYellow() {
    return cv::Point(yellow_x / yellow_num, yellow_y / yellow_num);
  }
  cv::Point getBlue() {
    return cv::Point(blue_x / blue_num, blue_y / blue_num);
  }
  cv::Point getGreen() {
    return cv::Point(green_x / green_num, green_y / green_num);
  }

private:
  int yellow_x = 0;
  int yellow_y = 0;
  int yellow_num = 0;

  int blue_x = 0;
  int blue_y = 0;
  int blue_num = 0;

  int green_x = 0;
  int green_y = 0;
  int green_num = 0;
};

int findBiggestBlob(const std::vector<cv::KeyPoint>& keypoints) {
  int max_idx = -1;
  int max_size = 0;
  for (size_t i = 0; i < keypoints.size(); ++i) {
    if (keypoints[i].size >= max_size) {
      max_size = keypoints[i].size;
      max_idx = i;
    }
  }
  return max_idx;
}

// Count pixels of each color
Blobs findBlobs(const cv::Mat& mat, cv::Mat& img_thresh) {
  int yellow_x = 0;
  int yellow_y = 0;
  int yellow_num = 0;

  int blue_x = 0;
  int blue_y = 0;
  int blue_num = 0;

  int green_x = 0;
  int green_y = 0;
  int green_num = 0;

  /* 
  - Scan all the pixels on the page
  - Categorise them as yellow, blue, or yellow
  */

  int height = mat.rows;
  int width = mat.cols;

  ROS_INFO("(Width: %d) (Height: %d)", width, height);
      
  int lowR = 120;
  int highR = 200;
  int lowG = 60;
  int highG = 120;
  int lowB = 0;
  int highB = 95;
  cv::inRange(mat, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), img_thresh);

  // Remove small crap
  cv::erode(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  // Fill in holes
  cv::dilate(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::erode(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  cv::SimpleBlobDetector::Params params;
  params.filterByArea = true;
  params.filterByCircularity = false;
  params.filterByColor = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;
  params.minArea = 50;
  params.maxArea = 20000;
  cv::Ptr<cv::SimpleBlobDetector> blobber = cv::SimpleBlobDetector::create(params);

  std::vector<cv::KeyPoint> keypoints;
  blobber->detect(img_thresh, keypoints);

/*  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      cv::Vec3b pixel = mat.at<cv::Vec3b>(j,i);
      
      // The input format is BGR
      
      // yellow
      if ((pixel[2] >= lowR && pixel[2] <= 200) && (pixel[1] > 60 && pixel[1] < 120) && (pixel[0] > 0 && pixel[0] < 95)){
        yellow_x += i;
        yellow_y += j;
        yellow_num += 1;
      }

      // blue 
      else if ((pixel[2] < 60) && (pixel[1] < 100) && (pixel[0] >= 140)){
        blue_x += i;
        blue_y += j;
        blue_num += 1;
      }*/
/*
      // green
      else if ((pixel[2] < 160) && (pixel[1] >= 150) && (pixel[0] < 130 )){
        green_x += i;
        green_y += j;
        green_num += 1;
      }
    }
  }*/

  Blobs blobs;
  int best_blob = findBiggestBlob(keypoints);
  if (best_blob >= 0) {
    yellow_x = keypoints[best_blob].pt.x;
    yellow_y = keypoints[best_blob].pt.y;
    yellow_num = 1;
  }
  blobs.setYellow(yellow_x, yellow_y, yellow_num);
  blobs.setBlue(blue_x, blue_y, blue_num);
  blobs.setGreen(green_x, green_y, green_num);
  return blobs;
}

bool calibrateSrv(example_nodes::CalibrateSrv::Request& req, example_nodes::CalibrateSrv::Response& res) {

  cv::namedWindow(OPENCV_WINDOW);

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
  // Avoid segmentation fault by not trying any later logic if empty frames
  // are being received.
  // (At end of for loop, iterator should equal max value...
  if (attempts_made == num_attempts) {
    return false;
  }

  // TODO: look into depth?
  cv::Mat &mat = cvImagePtr -> image;
  // cv::Mat &mat2 = dpImagePtr ->image;

  res.found = true;
  res.points.clear();
  std::vector<cv::Point2d> pointbuf;
  bool found = cv::findCirclesGrid(mat, cv::Size(5, 6), pointbuf);
  ROS_INFO("Found: %d", found ? 1 : 0);
  if (found) {
    for (size_t i = 0; i < pointbuf.size(); ++i) {
      cv::circle(mat, pointbuf[i], 2, CV_RGB(0,255,255), 2);
      cv::circle(mat, pointbuf[i], 20, CV_RGB(0,255,255), 1);
      geometry_msgs::Point pt;
      pt.x = pointbuf[i].x;
      pt.y = pointbuf[i].y;
      pt.z = 0;
      res.points.push_back(geometry_msgs::Point(pt));
    }
  }

  cv::imshow(OPENCV_WINDOW, mat);
  cv::waitKey(2);

  return true; 
}

bool visionSrv(example_nodes::VisionSrv::Request& req, example_nodes::VisionSrv::Response& res) {
  cv::namedWindow(OPENCV_WINDOW);

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
  // Avoid segmentation fault by not trying any later logic if empty frames
  // are being received.
  // (At end of for loop, iterator should equal max value...
  if (attempts_made == num_attempts) {
    return false;
  }

  // TODO: look into depth?
  cv::Mat &mat = cvImagePtr -> image;
  // cv::Mat &mat2 = dpImagePtr ->image;

  cv::Mat img_thresh;
  Blobs blobs = findBlobs(mat, img_thresh);

  // TODO: with new color structure, iterate through following options...even get
  // "best" (biggest, right number of pixels), and then just do that?
  if (blobs.hasYellow()) {
    auto yellow = blobs.getYellow();        
    cv::circle(img_thresh, yellow, 2, CV_RGB(0,255,255), 3);
    cv::circle(img_thresh, yellow, 20, CV_RGB(0,255,255), 2);
    res.x = yellow.x;
    res.y = yellow.y;
    // TODO: part of new "color" structure?
    res.r = 255;
    res.g = 255;
    res.b = 0;
    // TODO: is this duplicate with return value?
    res.found = true;
  }
/*
  else if (blobs.hasBlue()) {
    auto blue = blobs.getBlue();        
    cv::circle(mat, blue, 2, CV_RGB(0,255,255), 3);
    cv::circle(mat, blue, 20, CV_RGB(0,255,255), 2);
    res.x = blue.x;
    res.y = blue.y;
    // TODO: part of new "color" structure?
    res.r = 0;
    res.g = 0;
    res.b = 255;
    // TODO: is this duplicate with return value?
    res.found = true;
  }

  else if (blobs.hasGreen()) {
    auto green = blobs.getGreen();        
    cv::circle(mat, green, 2, CV_RGB(0,255,255), 3);
    cv::circle(mat, green, 20, CV_RGB(0,255,255), 2);
    res.x = green.x;
    res.y = green.y;
    // TODO: part of new "color" structure?
    res.r = 0;
    res.g = 255;
    res.b = 0;
    // TODO: is this duplicate with return value?
    res.found = true;
  }*/

  cv::imshow(OPENCV_WINDOW, img_thresh);
  cv::waitKey(2);

  return true; 
}

// Save image
void image_callback(sensor_msgs::Image data) {
  input_image = data;  
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "vision_process");

  ros::NodeHandle node;

  // Get camera images
  ros::Subscriber image_subscriber = node.subscribe("/camera/color/image_raw", 60, image_callback);

  // React to requests for finding objects and for calibration, both
  // using last received image.
  ros::ServiceServer vision_srv =
    node.advertiseService<example_nodes::VisionSrv::Request, example_nodes::VisionSrv::Response>(
      "/rosie/vision", &visionSrv);
  ros::ServiceServer calibrate_srv =
    node.advertiseService<example_nodes::CalibrateSrv::Request, example_nodes::CalibrateSrv::Response>(
      "/rosie/calibrate_vision", &calibrateSrv);


  ros::spin();
  return 0;
}

