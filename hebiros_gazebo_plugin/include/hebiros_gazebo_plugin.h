#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"

#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SetCommandLifetimeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"

#include "hebiros_gazebo_group.h"
#include "hebiros_gazebo_joint.h"
#include "hebiros_gazebo_controller.h"

using namespace hebiros;
using namespace gazebo;

class HebirosGazeboPlugin: public ModelPlugin {

public:
  HebirosGazeboPlugin() = default;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & _info);

private:

  physics::ModelPtr model;
  event::ConnectionPtr update_connection;
  std::map<std::string, std::shared_ptr<HebirosGazeboGroup>> hebiros_groups;
  std::map<std::string, std::shared_ptr<HebirosGazeboJoint>> hebiros_joints;

  std::string robot_namespace;
  std::shared_ptr<ros::NodeHandle> n;
  ros::Subscriber command_sub;
  ros::ServiceServer add_group_srv;
  ros::ServiceServer acknowledge_srv;

  void AddJointToGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group, std::string joint_name);
  void UpdateGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group, const ros::Duration& iteration_time);

  bool SrvAddGroup(AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res);

};
