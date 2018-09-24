#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <example_nodes/ArmMotionAction.h>
#include <example_nodes/GripperSrv.h>

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
    return (
      location.x > 0.1 && location.x < 0.4 &&
      location.y > -0.2 && location.y < 0.2 &&
      location.z > -0.11 && location.z < -0.09);
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
    setGoalLocation({0.3, 0.0, 0.3});
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

  Base(ros::NodeHandle& node) { }

  bool rotate(double radians) {
    // TODO: implement
    return true;
  }

  bool moveTo(const Location& location) {
    // TODO: implement
    return true;
  }
};

class Vision {
public:
  struct Location {
    double x;
    double y;
  };

  Vision(ros::NodeHandle& node) { }

  bool search(Location& found_at) {
    // TODO: implement!
    return true;
    //return false;
  }
};

// Note -- could do this in a super fancy class template abstract/general way
// with a "LocationTransformer" static template class...
// for now, we just hardcode transformations from Vision to Arm and base here...
Arm::Location transformToArm(const Vision::Location& source)
{
  // TODO: implement
  return Arm::Location{0.3, 0.0, -0.1};
}
Base::Location transformToBase(const Vision::Location& source)
{
  // TODO: implement
  return Base::Location{0.3, 0.0};
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
    // TODO: additional logic here to not just run over the friggin' thing
    base.moveTo(transformToBase(location)); 

    // TODO: is this even necessary if I'm just using actions and services?
    ros::spinOnce();
  }

  return 0;
}

