#pragma once

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/SettingsMsg.h"
#include "hebiros/SetCommandLifetimeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include "joint.h"

namespace hebi {
namespace sim {
namespace plugin {

class HebirosGazeboGroup : public std::enable_shared_from_this<HebirosGazeboGroup> {

public:
  HebirosGazeboGroup(std::string name,
    const std::vector<hebi::sim::Joint*>& joints,
    std::shared_ptr<ros::NodeHandle> n);

  void UpdateFeedback(const ros::Duration& iteration_time);

  ros::Time GetPrevTime() { return prev_time_; }
  void SetPrevTime(ros::Time t) { prev_time_ = t; }

private:
  std::string name_;
  hebiros::FeedbackMsg feedback_;
  bool check_acknowledgement_ = false;
  bool acknowledgement_ = false;
  int command_lifetime_ = 100;
  int feedback_frequency_ = 100;

  ros::Time start_time_;
  ros::Time prev_feedback_time_;
  ros::Time prev_time_;

  ros::Subscriber command_sub_;
  ros::Publisher feedback_pub_;
  ros::ServiceServer acknowledge_srv_;
  ros::ServiceServer command_lifetime_srv_;
  ros::ServiceServer feedback_frequency_srv_;

  void SubCommand(const boost::shared_ptr<hebiros::CommandMsg const> data);
  bool SrvAcknowledge(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool SrvSetCommandLifetime(hebiros::SetCommandLifetimeSrv::Request &req, hebiros::SetCommandLifetimeSrv::Response &res);
  bool SrvSetFeedbackFrequency(hebiros::SetFeedbackFrequencySrv::Request &req, hebiros::SetFeedbackFrequencySrv::Response &res);

  std::vector<hebi::sim::Joint*> joints_;
};

}
}
}
