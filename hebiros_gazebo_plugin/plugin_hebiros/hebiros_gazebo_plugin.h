#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"

#include "hebi_gazebo_plugin.h"

#include "hebiros/AddGroupFromNamesSrv.h"

#include "hebiros_gazebo_group.h"

namespace hebi {
namespace sim {
namespace plugin {

class HebirosGazeboPlugin : public HebiGazeboPlugin {

public:
  HebirosGazeboPlugin() = default;
  
  void onLoad(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  void onUpdate(const gazebo::common::UpdateInfo& info) override;

private:

  bool first_sim_iteration_{true};  
  std::map<std::string, std::shared_ptr<HebirosGazeboGroup>> hebiros_groups_;
  // To remove the ROS dependency from the hebi::sim::Joint class, we keep
  // the IMU subscriptions here.
  std::vector<ros::Subscriber> hebiros_joint_imu_subs_;

  std::string robot_namespace_;
  std::shared_ptr<ros::NodeHandle> n_;
  ros::Subscriber command_sub_;
  ros::ServiceServer add_group_srv_;
  ros::ServiceServer acknowledge_srv_;

  bool addGroupSrv(hebiros::AddGroupFromNamesSrv::Request &req, hebiros::AddGroupFromNamesSrv::Response &res);
};

}
}
}