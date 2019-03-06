#include <hebiros_gazebo_group.h>
#include "hebiros_gazebo_controller.h"

HebirosGazeboGroup::HebirosGazeboGroup(std::string name,
  std::shared_ptr<ros::NodeHandle> n) {
  this->name = name;

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;
  this->prev_feedback_time = current_time;

  this->command_sub = n->subscribe<CommandMsg>("hebiros_gazebo_plugin/command/"+name, 100,
    boost::bind(&HebirosGazeboGroup::SubCommand, this, _1));

  this->acknowledge_srv =
    n->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    "hebiros_gazebo_plugin/acknowledge/"+name, boost::bind(
    &HebirosGazeboGroup::SrvAcknowledge, this, _1, _2));

  this->command_lifetime_srv =
    n->advertiseService<SetCommandLifetimeSrv::Request, SetCommandLifetimeSrv::Response>(
    "hebiros_gazebo_plugin/set_command_lifetime/"+name, boost::bind(
    &HebirosGazeboGroup::SrvSetCommandLifetime, this, _1, _2));

  this->feedback_frequency_srv =
    n->advertiseService<SetFeedbackFrequencySrv::Request, SetFeedbackFrequencySrv::Response>(
    "hebiros_gazebo_plugin/set_feedback_frequency/"+name, boost::bind(
    &HebirosGazeboGroup::SrvSetFeedbackFrequency, this, _1, _2));
}

void HebirosGazeboGroup::SubCommand(const boost::shared_ptr<CommandMsg const> data) {

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

      HebirosGazeboController::ChangeSettings(shared_from_this(),
        hebiros_joint);
    }
  }
}

//Service callback which acknowledges that a command has been received
bool HebirosGazeboGroup::SrvAcknowledge(std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res) {

  this->check_acknowledgement = true;

  if (this->acknowledgement) {
    this->check_acknowledgement = false;
    this->acknowledgement = false;
    return true;
  }
  else {
    return false;
  }
}

bool HebirosGazeboGroup::SrvSetCommandLifetime(SetCommandLifetimeSrv::Request &req,
  SetCommandLifetimeSrv::Response &res) {

  command_lifetime = req.command_lifetime;
  return true;
}

bool HebirosGazeboGroup::SrvSetFeedbackFrequency(SetFeedbackFrequencySrv::Request &req,
  SetFeedbackFrequencySrv::Response &res) {

  feedback_frequency = req.feedback_frequency;
  return true;
}






