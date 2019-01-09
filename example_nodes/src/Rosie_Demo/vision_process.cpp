
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

enum class ColorSpace {
  RGB,
  HSV
};

struct ColorParams {
  // LED display - what is shown on the robot when moving to
  // or picking up that color of beanbag
  uint8_t r{};
  uint8_t g{};
  uint8_t b{};
  // Thresholds
  uint8_t lowRH{}; // Red or Hue
  uint8_t highRH{};
  uint8_t lowGS{}; // Green or Saturation
  uint8_t highGS{};
  uint8_t lowBV{}; // Blue or Value
  uint8_t highBV{};
  // RGB, HSV, etc.
  ColorSpace colorSpace{};
  // Lower priorities are chosen first
  int priority{};

  // Try setting the values from a value read from the ROS
  // parameter server.
  // The format is:
  //
  // rdisp: <uint8_t>
  // gdisp: <uint8_t>
  // bdisp: <uint8_t>
  // priority: <int>
  //
  // and then for RGB, additionally:
  // rmin: <uint8_t>
  // rmax: <uint8_t>
  // gmin: <uint8_t>
  // gmax: <uint8_t>
  // bmin: <uint8_t>
  // bmax: <uint8_t>
  //
  // and for HSV, additionally:
  // hmin: <uint8_t>
  // hmax: <uint8_t>
  // smin: <uint8_t>
  // smax: <uint8_t>
  // vmin: <uint8_t>
  // vmax: <uint8_t>
  bool trySetFromROSParam(std::map<std::string, int>& params) {
    // Note: we don't check the range of these values, but
    // we could/probably should
    if (params.count("rdisp") == 0 ||
        params.count("gdisp") == 0 ||
        params.count("bdisp") == 0 ||
        params.count("priority") == 0) {
      return false;
    }

    if (params.count("rmin") == 1 &&
        params.count("rmax") == 1 &&
        params.count("gmin") == 1 &&
        params.count("gmax") == 1 &&
        params.count("bmin") == 1 &&
        params.count("bmax") == 1) {
      lowRH = (uint8_t)params["rmin"];
      highRH = (uint8_t)params["rmax"];
      lowGS = (uint8_t)params["gmin"];
      highGS = (uint8_t)params["gmax"];
      lowBV = (uint8_t)params["bmin"];
      highBV = (uint8_t)params["bmax"];
      colorSpace = ColorSpace::RGB;
    } else if (params.count("hmin") == 1 &&
        params.count("hmax") == 1 &&
        params.count("smin") == 1 &&
        params.count("smax") == 1 &&
        params.count("vmin") == 1 &&
        params.count("vmax") == 1) {
      lowRH = (uint8_t)params["hmin"];
      highRH = (uint8_t)params["hmax"];
      lowGS = (uint8_t)params["smin"];
      highGS = (uint8_t)params["smax"];
      lowBV = (uint8_t)params["vmin"];
      highBV = (uint8_t)params["vmax"];
      colorSpace = ColorSpace::HSV;
    }

    r = (uint8_t)params["rdisp"];
    g = (uint8_t)params["gdisp"];
    b = (uint8_t)params["bdisp"];
    priority = (uint8_t)params["priority"];
    return true;
  }
};

std::vector<ColorParams> enabledColors;

class Blob {
public:
  Blob(int x, int y) : x_(x), y_(y), has_blob_(true) {}
  Blob() = default;
  int x_ {0};
  int y_ {0};
  bool has_blob_{false};
};

Blob getBlob(const cv::Mat& mat, cv::Mat& img_thresh, const ColorParams& color_params) {

  int height = mat.rows;
  int width = mat.cols;

  // Threshold the image  
  cv::inRange(mat,
    cv::Scalar((int)color_params.lowBV, (int)color_params.lowGS, (int)color_params.lowRH),
    cv::Scalar((int)color_params.highBV, (int)color_params.highGS, (int)color_params.highRH), img_thresh);

  // Remove small crap
  cv::erode(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  // Fill in holes
  cv::dilate(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::erode(img_thresh, img_thresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  // Detect blobs
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
  res.found = false;

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
      ROS_WARN("Failed to load stream. Trying again...");
    }
  }
  // Avoid segmentation fault by not trying any later logic if empty frames
  // are being received.
  // (At end of for loop, iterator should equal max value...
  if (attempts_made == num_attempts) {
    return false;
  }
  ROS_INFO("Got Stream");

  // TODO: look into depth?
  cv::Mat &mat = cvImagePtr -> image;

  res.points.clear();
  std::vector<cv::Point2d> pointbuf;

  // Note: if detection isn't working particularly well, you can tweak these
  // values to ensure the blocks are appearing:
  cv::SimpleBlobDetector::Params params;
//  params.minArea = 20;
//  params.filterByColor = false;
//  params.filterByCircularity = false;
//  params.filterByConvexity = false;
  cv::Ptr<cv::SimpleBlobDetector> blobber = cv::SimpleBlobDetector::create(params);

  // DEBUGGING - see what blobs are actually being detected + fed into
  // findCirclesGrid; draw with purple circles.
  std::vector<cv::KeyPoint> keypoints;
  blobber->detect(mat, keypoints);
  for (size_t i = 0; i < keypoints.size(); ++i) {
    cv::circle(mat, keypoints[i].pt, 10, CV_RGB(255,0,255), 2);   
  }
  ROS_WARN_STREAM("Keypoints size: " << keypoints.size());

  // Now, actually find the point cloud.  "CB_CLUSTERING" is better with
  // distortion, but more susceptible to noise.
  bool found = cv::findCirclesGrid(mat, cv::Size(5, 6), pointbuf,
    cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobber);
  res.found = found;
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
      ROS_WARN("Failed to load stream. Trying again...");
    }
  }
  // Avoid segmentation fault by not trying any later logic if empty frames
  // are being received.
  // (At end of for loop, iterator should equal max value...
  if (attempts_made == num_attempts) {
    return false;
  }

  // TODO: look into depth?
  cv::Mat &img_rgb = cvImagePtr -> image;
  cv::Mat img_hsv;
  cv::cvtColor(img_rgb, img_hsv, cv::COLOR_BGR2HSV); // convert from rgb to hsv
  cv::Mat img_thresh;

  // We check for each color we want to consider here, in a priority order.
  Blob blob;
  for (const auto& color : enabledColors)
  {
    if (color.colorSpace == ColorSpace::RGB)
      blob = getBlob(img_rgb, img_thresh, color);
    else if (color.colorSpace == ColorSpace::HSV)
      blob = getBlob(img_hsv, img_thresh, color);
   
    if (blob.has_blob_)
    {
      // Save RGB values to display on the LEDs and don't
      // check anymore
      res.r = color.r;
      res.g = color.g;
      res.b = color.b;
      break; 
    }
  }

  if (blob.has_blob_) {
    cv::Point2d pt(blob.x_, blob.y_);
    cv::circle(img_thresh, pt, 2, CV_RGB(0,255,255), 3);
    cv::circle(img_thresh, pt, 20, CV_RGB(0,255,255), 2);

    res.x = pt.x;
    res.y = pt.y;
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

  // Get colors from parameter server:
  std::vector<std::string> color_names;
  node.getParam("rosie/enabled_colors", color_names);
  for (const auto& color_name : color_names) {
   
    std::map<std::string, int> params;
    node.getParam("rosie/" + color_name, params);
    ColorParams new_color; 
    if (new_color.trySetFromROSParam(params)) {
      // Find insert location (sorted by priority)
      auto insert_location = std::find_if(
        enabledColors.begin(), enabledColors.end(),
        [&new_color](const ColorParams& existing) {
          return existing.priority > new_color.priority;
        });
      // Insert in list
      enabledColors.insert(insert_location, new_color);
    } 
  }

  ROS_INFO_STREAM("Loaded " << enabledColors.size() << " colors of bean bags to search for.");

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

