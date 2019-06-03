#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"

#include "hebi_gazebo_plugin.h"

#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SetCommandLifetimeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"

#include "hebiros_gazebo_group.h"

namespace hebi {
namespace sim {
namespace plugin {

class HebirosGazeboPlugin : public HebiGazeboPlugin {

public:
  HebirosGazeboPlugin() = default;
  
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  void OnUpdate(const gazebo::common::UpdateInfo & info);

private:

  bool first_sim_iteration{true};  
  gazebo::event::ConnectionPtr update_connection;
  std::map<std::string, std::shared_ptr<HebirosGazeboGroup>> hebiros_groups;
  // To remove the ROS dependency from the hebi::sim::Joint class, we keep
  // the IMU subscriptions here.
  std::vector<ros::Subscriber> hebiros_joint_imu_subs;

  std::string robot_namespace;
  std::shared_ptr<ros::NodeHandle> n;
  ros::Subscriber command_sub;
  ros::ServiceServer add_group_srv;
  ros::ServiceServer acknowledge_srv;

  bool SrvAddGroup(hebiros::AddGroupFromNamesSrv::Request &req, hebiros::AddGroupFromNamesSrv::Response &res);
};

}
}
}