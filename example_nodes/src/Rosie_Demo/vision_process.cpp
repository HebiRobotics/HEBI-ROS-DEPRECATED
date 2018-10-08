
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

struct ColorParams {
  // LED display
  uint8_t r;
  uint8_t g;
  uint8_t b;
  // Thresholds
  int lowR;
  int highR;
  int lowG;
  int highG;
  int lowB;
  int highB;
};

const ColorParams YellowParamsRGB {
  255, 255, 0,
  120, 200,
  60, 120,
  0, 95
};

const ColorParams YellowParamsHSV {
  255, 255, 0,
  122, 213,
  72, 213,
  0, 75
};

const ColorParams GreenParams {
  0, 255, 0,
  0, 66,
  55, 101,
  37, 101
};

const ColorParams RedParamsRGB {
  255, 0, 0,
  103, 255,
  0, 47,
  0, 64
};
// HSV
const ColorParams RedParamsHSV {
  255, 0, 0,
  103, 181,
  136, 255,
  145, 205
};

const ColorParams PurpleParams {
  255, 0, 255,
  12, 79,
  12, 40,
  42, 86
};

class Blob {
public:
  Blob(int x, int y) : x_(x), y_(y), has_blob_(true) {}
  Blob() = default;
  int x_ {0};
  int y_ {0};
  bool has_blob_{false};
};

// Count pixels of each color
Blob getBlob(const cv::Mat& mat, cv::Mat& img_thresh, const ColorParams& color_params) {
  /* 
  - Scan all the pixels on the page
  - Categorise them as yellow, blue, or yellow
  */

  int height = mat.rows;
  int width = mat.cols;

  ROS_INFO("(Width: %d) (Height: %d)", width, height);
  
  // Yellow:    
  cv::inRange(mat,
    cv::Scalar(color_params.lowB, color_params.lowG, color_params.lowR),
    cv::Scalar(color_params.highB, color_params.highG, color_params.highR), img_thresh);

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
  params.minArea = 1000;
  params.maxArea = 20000;
  cv::Ptr<cv::SimpleBlobDetector> blobber = cv::SimpleBlobDetector::create(params);

  std::vector<cv::KeyPoint> keypoints;
  blobber->detect(img_thresh, keypoints);

  int best_blob = findBiggestBlob(keypoints);
  if (best_blob >= 0)
    return Blob(keypoints[best_blob].pt.x, keypoints[best_blob].pt.y);
  
  return Blob();
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
  cv::Mat img_hsv;
  cv::cvtColor(mat, img_hsv, cv::COLOR_BGR2HSV); // convert from rgb to hsv

  cv::Mat img_thresh;
  auto params = YellowParamsHSV;
  Blob blob = getBlob(img_hsv, img_thresh, params);

  if (!blob.has_blob_) {
    params = RedParamsHSV;
    blob = getBlob(img_hsv, img_thresh, params);
  }

  if (blob.has_blob_) {
    cv::Point2d pt(blob.x_, blob.y_);
    cv::circle(img_thresh, pt, 2, CV_RGB(0,255,255), 3);
    cv::circle(img_thresh, pt, 20, CV_RGB(0,255,255), 2);

    res.x = pt.x;
    res.y = pt.y;
    res.r = params.r;
    res.g = params.g;
    res.b = params.b;
    // TODO: is this duplicate with return value?
    res.found = true;
  }

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

