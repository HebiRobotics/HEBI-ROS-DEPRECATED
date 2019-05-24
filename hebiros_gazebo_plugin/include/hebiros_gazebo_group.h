#pragma once

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/SettingsMsg.h"
#include "hebiros/SetCommandLifetimeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include "joint.h"

using namespace hebiros;

class HebirosGazeboGroup : public std::enable_shared_from_this<HebirosGazeboGroup> {

public:
  void UpdateFeedback(const ros::Duration& iteration_time);

  // TODO: Make these private.
  std::string name;
  FeedbackMsg feedback;
  bool check_acknowledgement = false;
  bool acknowledgement = false;
  int command_lifetime = 100;
  int feedback_frequency = 100;

  ros::Time start_time;
  ros::Time prev_time;
  ros::Time prev_feedback_time;

  ros::Subscriber command_sub;
  ros::Publisher feedback_pub;
  ros::ServiceServer acknowledge_srv;
  ros::ServiceServer command_lifetime_srv;
  ros::ServiceServer feedback_frequency_srv;

  HebirosGazeboGroup(std::string name,
    const std::vector<hebi::sim::Joint*>& joints_,
    std::shared_ptr<ros::NodeHandle> n);

  void SubCommand(const boost::shared_ptr<CommandMsg const> data);
  bool SrvAcknowledge(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool SrvSetCommandLifetime(SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res);
  bool SrvSetFeedbackFrequency(SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res);

private:
  std::vector<hebi::sim::Joint*> joints;
};
