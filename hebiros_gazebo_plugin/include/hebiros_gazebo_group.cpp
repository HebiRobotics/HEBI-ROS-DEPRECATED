
#include <hebiros_gazebo_group.h>

HebirosGazeboGroup::HebirosGazeboGroup(std::string name,
  std::shared_ptr<ros::NodeHandle> n) {
  this->name = name;

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;
  this->prev_feedback_time = current_time;

  this->command_sub = n->subscribe<CommandMsg>("/hebiros_gazebo_plugin/command/"+name, 100,
    boost::bind(&HebirosGazeboGroup::SubCommand, this, _1));
}

HebirosGazeboGroup::~HebirosGazeboGroup() {}

HebirosGazeboGroup::SubCommand(const boost::shared_ptr<CommandMsg const> data) {

  if (this->check_acknowledgement) {
    this->acknowledgement = true;
    this->check_acknowledgement = false;
  }

  if (!this->command_received) {
    this->command_received = true;
  }

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;

  command_target = *data;

  for (int i = 0; i < data->name.size(); i++) {
    std::string joint_name = data->name[i];

    if (joints.find(joint_name) != joints.end()) {
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint = joints[joint_name];
      hebiros_joint->command_index = i;

      HebirosGazeboController::ChangeSettings(hebiros_group, hebiros_joint);
    }
  }
}

