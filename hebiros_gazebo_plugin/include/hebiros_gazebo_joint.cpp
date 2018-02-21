
#include <hebiros_gazebo_joint.h>


HebirosGazeboJoint::HebirosGazeboJoint(std::string name) {
  this->name = name;
}

HebirosGazeboJoint::~HebirosGazeboJoint() {}

void HebirosGazeboJoint::Reset(int i, CommandMsg command_msg) {

  this->command_index = i;
  this->command_target = command_msg;

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;
}
