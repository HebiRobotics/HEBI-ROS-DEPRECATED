#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <example_nodes/ArmMotionAction.h>
#include <example_nodes/BaseMotionAction.h>
#include <example_nodes/GripperSrv.h>
#include <example_nodes/VisionSrv.h>
#include <example_nodes/CalibrateSrv.h>

#include "Eigen/Core"

// We abstract the behavior of each of the core components up here, so our
// main logic loop doesn't have to deal with ros services, actions, etc.

class Gripper {
public:
  Gripper(ros::NodeHandle& node)
    : client_(node.serviceClient<example_nodes::GripperSrv>("/rosie/gripper")) {
  }

  bool close() {
    message_.request.closed = true;
    if (client_.call(message_)) {
      ros::Duration(0.75).sleep();
      return true;
    }
    return false;
  }

  bool open() {
    message_.request.closed = false;
    if (client_.call(message_)) {
      ros::Duration(1).sleep();
      return true;
    }
    return false;
  }

private:
  ros::ServiceClient client_;
  example_nodes::GripperSrv message_;
};

class Arm {
public:
  struct Location {
    double x;
    double y;
    double z;
  };

  Arm(ros::NodeHandle& node)
    : gripper_(node), arm_motion_("/rosie/arm_motion", true) {
    // Wait for the server to start (can wait forever)
    arm_motion_.waitForServer();
  }

  bool canReach(const Location& location) {
    // TODO: check bounds are reasonable
    bool can_reach = 
      location.x > 0.25 && location.x < 0.55 &&
      location.y > -0.2 && location.y < 0.2 &&
      location.z > -0.11 && location.z < -0.09;
    if (can_reach)
      ROS_INFO("Can reach %f %f %f", location.x, location.y, location.z);
    else
      ROS_INFO("Cannot reach %f %f %f", location.x, location.y, location.z);
    return can_reach;
  }

  bool moveHome() {
    setGoalHome();
    if (!moveToGoal())
      return false;
  }

  bool pickup(const Location& location) {
    // Call service to move to pick up location, facing down:
    setGoalLocation(location);
    setGoalTipDown();
    if (!moveToGoal())
      return false;

    // Pick up with gripper
    if (!gripper_.close())
      return false;

    // TODO: smooth these motions into a single action! Need additional functionality from RosieArmNode...
    // Call service to move home
    setGoalHome();
    if (!moveToGoal())
      return false;
    // Call service to move to drop position
    setGoalDrop();
    if (!moveToGoal())
      return false;

    if (!gripper_.open())
      return false;

    // Call service to move home
    setGoalHome();
    if (!moveToGoal())
      return false;
  }

private:

  // TODO: could refactor so this takes in argument, and "setGoal" etc. functions
  // below are just static consts instead...
  bool moveToGoal() {
    arm_motion_.sendGoal(arm_motion_goal_);

    // wait for the action to return
    bool finished_before_timeout = arm_motion_.waitForResult(ros::Duration(15.0));

    if (!finished_before_timeout) {
      ROS_ERROR("Arm motion action timed out");
      return false;
    }

    auto res = arm_motion_.getState();
    ROS_INFO("Arm motion action finished: %s", res.toString().c_str());
    return true;
  }

  void setGoalLocation(const Location& location) {
    arm_motion_goal_.x = location.x;
    arm_motion_goal_.y = location.y;
    arm_motion_goal_.z = location.z;
  }

  void setGoalHome() {
    setGoalLocation({0.2, -0.2, 0.3});
    //setGoalTipForward();
    setGoalTipDown();
  }

  void setGoalDrop() {
    setGoalLocation({-0.1, -0.2, 0.3});
    setGoalTipDown();
  }

  void setGoalTipDown() {
    arm_motion_goal_.tipx = 0;
    arm_motion_goal_.tipy = 0;
    arm_motion_goal_.tipz = -1;
  }

  void setGoalTipForward() {
    arm_motion_goal_.tipx = 1;
    arm_motion_goal_.tipy = 0;
    arm_motion_goal_.tipz = 0;
  }

  Gripper gripper_;

  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<example_nodes::ArmMotionAction> arm_motion_;

  example_nodes::ArmMotionGoal arm_motion_goal_;
};

class Base {
public:
  struct Location {
    double x;
    double y;
  };

  Base(ros::NodeHandle& node)
    : base_motion_("/rosie/base_motion", true) {
    // Wait for the server to start (can wait forever)
    base_motion_.waitForServer();
  }

  bool rotate(double radians) {
    base_motion_goal_.x = 0.0;
    base_motion_goal_.y = 0.0;
    base_motion_goal_.theta = radians; 

    base_motion_.sendGoal(base_motion_goal_);

    // wait for the action to return
    bool finished_before_timeout = base_motion_.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout) {
      ROS_ERROR("Base motion action timed out");
      return false;
    }

    auto res = base_motion_.getState();
    ROS_INFO("Base motion action finished: %s", res.toString().c_str());
    return true;
  }

  bool moveTo(const Location& location) {
    
    // Logic here to not just run over the friggin' thing
    base_motion_goal_.theta = atan2(location.y, location.x);
    double len = std::sqrt(location.x * location.x + location.y * location.y);
    double actual_len = len - 0.3556; // stay 14 inches away
    if (actual_len < 0)
      actual_len = 0;
    double frac = actual_len / len;
    base_motion_goal_.x = location.x * frac;
    base_motion_goal_.y = location.y * frac;
    ROS_INFO("Base motion moving: %f %f %f", base_motion_goal_.x, base_motion_goal_.y, base_motion_goal_.theta);

    base_motion_.sendGoal(base_motion_goal_);

    // wait for the action to return
    bool finished_before_timeout = base_motion_.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout) {
      ROS_ERROR("Base motion action timed out");
      return false;
    }

    auto res = base_motion_.getState();
    ROS_INFO("Base motion action finished: %s", res.toString().c_str());
    return true;
  }

  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<example_nodes::BaseMotionAction> base_motion_;

  example_nodes::BaseMotionGoal base_motion_goal_;
};

// This is an "OK" solution...
double affine_transform[2][3]
  { { -0.0015, -0.0013,  1.5138 },
    { -0.0007,  0.0017, -0.1990 } };

class Vision {
public:
  struct Location {
    double x;
    double y;
  };

  Vision(ros::NodeHandle& node)
    : client_(node.serviceClient<example_nodes::VisionSrv>("/rosie/vision")),
      calibrate_client_(node.serviceClient<example_nodes::CalibrateSrv>("/rosie/calibrate_vision")) {
  }

  bool search(Location& found_at) {
    // Got a success back?
    if (client_.call(message_) && message_.response.found) {
      found_at.x = message_.response.x;
      found_at.y = message_.response.y;
      return true;
    }
    return false;
  }

  // Collect:
  //   a: 3xn matrix of points in camera frame, with row of '1' at the bottom
  //   b: 3xn matrix of points in world frame (in meters) that correspond, with row of '1' at the bottom.
  // Compute this in MATLAB (basically, using least squares solution):
  //   X = b / a
  // or equivalently
  //   X = a' \ b'
  bool calibrate() {
    if (calibrate_client_.call(calibrate_message_) && calibrate_message_.response.found) {
      // Go through all points, do SVD!
      // The 36 points should be row-by-row, given some basic testing
      Eigen::MatrixXd camera_pts(3, 36);
      if (calibrate_message_.response.points.size() != 36)
        return false;
      for (size_t i = 0; i < calibrate_message_.response.points.size(); ++i) {
        camera_pts(0, i) = calibrate_message_.response.points[i].x;
        camera_pts(1, i) = calibrate_message_.response.points[i].y;
        camera_pts(0, i) = 1.0;
      }

      // The points they should match up with:
      Eigen::MatrixXd world_pts(3, 36);
      double spacing = 0.075; // mm between row centers
      double top_row = 0.15 + 0.54; // 15 cm from center to front of robot, 54 cm to top row on paper.
      double left_col = 2.5 * spacing; // 2.5 rows to the left
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          world_pts(0, row * 6 + col) = top_row - (double)row * spacing;
          world_pts(1, row * 6 + col) = left_col + (double)col * spacing;
          world_pts(2, row * 6 + col) = 1.0;
        }
      }

      // TODO: UPDATE MATRIX USING SVD HERE!!!!

      return true;
    }
    return false;
  }

private:
  ros::ServiceClient client_;
  example_nodes::VisionSrv message_;
  ros::ServiceClient calibrate_client_;
  example_nodes::CalibrateSrv calibrate_message_;
};

// Note -- could do this in a super fancy class template abstract/general way
// with a "LocationTransformer" static template class...
// for now, we just hardcode transformations from Vision to Arm and base here...
Arm::Location transformToArm(const Vision::Location& source)
{
  double x = source.x * affine_transform[0][0] + source.y * affine_transform[0][1] + affine_transform[0][2];
  double y = source.x * affine_transform[1][0] + source.y * affine_transform[1][1] + affine_transform[1][2];
  return Arm::Location{x, y, -0.1};
}

Base::Location transformToBase(const Vision::Location& source)
{
  double x = source.x * affine_transform[0][0] + source.y * affine_transform[0][1] + affine_transform[0][2];
  double y = source.x * affine_transform[1][0] + source.y * affine_transform[1][1] + affine_transform[1][2];
  return Base::Location{x, y};
}

int main(int argc, char ** argv) {

  ros::init(argc, argv, "demo_central");

  ros::NodeHandle node;

  constexpr double command_rate = 100; // Hz
  ros::Rate loop_rate(command_rate);

  constexpr double rotate_increment = M_PI / 3.0; // 1/6 of a full rotation

  // Initialize abstracted components and their ROS interfaces
  Arm arm(node);
  Base base(node);
  Vision vision(node);
  Vision::Location location;

  arm.moveHome();

  // Run main logic
  while (ros::ok()) {
    // Can't find anything? rotate and continue the search
    if (!vision.search(location)) {
      base.rotate(rotate_increment);
      continue;
    }

    // Can the arm reach this? If so, retrieve, then continue the search
    auto arm_location = transformToArm(location);
    if (arm.canReach(arm_location)) {
      arm.pickup(arm_location);
      continue;
    }

    // Otherwise, go there with the base and then continue the search
    base.moveTo(transformToBase(location)); 

    // TODO: is this even necessary if I'm just using actions and services?
    ros::spinOnce();
  }

  return 0;
}

