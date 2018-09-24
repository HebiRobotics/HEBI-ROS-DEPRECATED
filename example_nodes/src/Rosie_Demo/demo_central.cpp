#include <ros/ros.h>
#include <ros/console.h>

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
    return client_.call(message_);
  }
  bool open() {
    message_.request.closed = false;
    return client_.call(message_);
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
    : gripper_(node) {
  }

  bool canReach(const Location& location) {
    // TODO: implement!
    return false;
  }

  bool pickup(const Location& location) {
    // TODO: call service to move to location
    if (!gripper_.close())
      return false;
    // TODO: call service to move home (TODO hardcode this!)
    // TODO: call service to move to drop position (TODO hardcode this!)
    // TODO: smooth this motion into a single action!
    if (!gripper_.open())
      return false;
    // TODO: call service to move home (TODO hardcode this!)
  }

private:

  Gripper gripper_;
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
    return false;
  }
};

// Note -- could do this in a super fancy class template abstract/general way
// with a "LocationTransformer" static template class...
// for now, we just hardcode transformations from Vision to Arm and base here...
Arm::Location transformToArm(const Vision::Location& source)
{
  // TODO: implement
  return Arm::Location{0.3, 0.0, 0.0};
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

    ros::spinOnce();
  }

  return 0;
}

