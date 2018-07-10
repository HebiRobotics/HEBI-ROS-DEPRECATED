#include "hebiros_subscribers.h"

#include "hebiros.h"


std::map<std::string, ros::Subscriber> HebirosSubscribers::subscribers;

bool HebirosSubscribers::jointFound(std::string group_name, std::string joint_name) {

  std::shared_ptr<HebirosGroup> group = HebirosGroup::getGroup(group_name);

  return group->joints.find(joint_name) != group->joints.end();
}

void HebirosSubscribers::jointNotFound(std::string joint_name) {
  ROS_WARN("Unable to find joint: %s.  Command will not be sent.", joint_name.c_str());
}

