#include "hebiros_gazebo_plugin.h"
#include "sensor_msgs/Imu.h"

namespace hebi {
namespace sim {
namespace plugin {

//Load the model and sdf from Gazebo
void HebirosGazeboPlugin::onLoad(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "hebiros_gazebo_plugin_node");

  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  if (robot_namespace_ == "") {
    n_.reset(new ros::NodeHandle);
  } else {
    n_.reset(new ros::NodeHandle(robot_namespace_));
  }

  ROS_INFO("Loaded hebiros gazebo plugin");
}

//Update the joints at every simulation iteration
void HebirosGazeboPlugin::onUpdate(const gazebo::common::UpdateInfo & info) {

  if (first_sim_iteration_) {
    first_sim_iteration_ = false;
    add_group_srv_ =
      n_->advertiseService<hebiros::AddGroupFromNamesSrv::Request, hebiros::AddGroupFromNamesSrv::Response>(
      "/hebiros_gazebo_plugin/add_group", boost::bind(
      &HebirosGazeboPlugin::addGroupSrv, this, _1, _2));
  }

  ros::Time current_time = ros::Time::now();

  // TODO: if we cache commands later for thread-safety, this is where we would read them and
  // set them on the joints.

  // Fill in the feedback:
  for (auto group_pair : hebiros_groups_) {
    auto hebiros_group = group_pair.second;

    // TODO: change this to update each module...?
    // Get the time elapsed since the last iteration
    ros::Duration iteration_time = current_time - hebiros_group->GetPrevTime();
    hebiros_group->SetPrevTime(current_time);
    hebiros_group->UpdateFeedback(iteration_time);
  }
}

//Service callback which adds a group with corresponding joints
bool HebirosGazeboPlugin::addGroupSrv(hebiros::AddGroupFromNamesSrv::Request &req,
  hebiros::AddGroupFromNamesSrv::Response &res) {

  if (hebiros_groups_.find(req.group_name) != hebiros_groups_.end()) {
    ROS_WARN("Group %s already exists", req.group_name.c_str());
    return true;
  }

  // Try to create a vector of joints:
  std::vector<hebi::sim::Joint*> joints;

  // TODO: eventually, implement full lookup with wildcards/etc;
  // document this process
  for (int i = 0; i < req.families.size(); i++) {
    for (int j = 0; j < req.names.size(); j++) {

      if ((req.families.size() == 1) ||
        (req.families.size() == req.names.size() && i == j)) {

        // Get a weak reference to store in the individual groups
        auto raw_joint = getJoint(req.families[i], req.names[j]);

        // This module couldn't be found!
        if (raw_joint == nullptr)
          return false;

        joints.push_back(raw_joint);
      }
    }
  }

  for (auto joint : joints)
  {
    // Temporarily, we store joint subscriptions in the gazebo ros plugin here, since
    // the IMU that generates this data is a separate ROS plugin communicating via ROS
    // messages.
    //
    // This is a big hack right now, and should be modified in the future, especially so
    // these is only one imu sub for a joint, even if it is in two groups.
    hebiros_joint_imu_subs_.push_back(n_->subscribe<sensor_msgs::Imu>(
      "hebiros_gazebo_plugin/imu/" + joint->getName(), 100, 
      [joint](const boost::shared_ptr<sensor_msgs::Imu const> data) {
        auto a = data->linear_acceleration;
        auto g = data->angular_velocity;
        joint->updateImu(
            {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)},
            {static_cast<float>(g.x), static_cast<float>(g.y), static_cast<float>(g.z)});
      }));
  }

  hebiros_groups_[req.group_name] = 
    std::make_shared<HebirosGazeboGroup>(req.group_name, joints, n_);

  return true;
}

}
}
}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(hebi::sim::plugin::HebirosGazeboPlugin);
