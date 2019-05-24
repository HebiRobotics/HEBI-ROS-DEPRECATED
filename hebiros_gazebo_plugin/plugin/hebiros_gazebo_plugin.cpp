#include "hebiros_gazebo_plugin.h"
#include "sensor_msgs/Imu.h"

//Load the model and sdf from Gazebo
void HebirosGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  HebiGazeboPlugin::Load(_model, _sdf);

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "hebiros_gazebo_plugin_node");

  this->robot_namespace = "";
  if (_sdf->HasElement("robotNamespace")) {
    this->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  if (this->robot_namespace == "") {
    this->n.reset(new ros::NodeHandle);
  } else {
    this->n.reset(new ros::NodeHandle(this->robot_namespace));
  }

  this->update_connection = event::Events::ConnectWorldUpdateBegin (
    boost::bind(&HebirosGazeboPlugin::OnUpdate, this, _1));

  ROS_INFO("Loaded hebiros gazebo plugin");
}

//Update the joints at every simulation iteration
void HebirosGazeboPlugin::OnUpdate(const common::UpdateInfo & info) {

  if (this->first_sim_iteration) {
    this->first_sim_iteration = false;
    this->add_group_srv =
      this->n->advertiseService<AddGroupFromNamesSrv::Request, AddGroupFromNamesSrv::Response>(
      "/hebiros_gazebo_plugin/add_group", boost::bind(
      &HebirosGazeboPlugin::SrvAddGroup, this, _1, _2));
  }

  ros::Time current_time = ros::Time::now();

  // TODO: if we cache commands later for thread-safety, this is where we would read them and
  // set them on the joints.

  // Update the feedback / controller for each joint in the simulation:
  OnUpdateBase(info);

  // Fill in the feedback:
  for (auto group_pair : hebiros_groups) {
    auto hebiros_group = group_pair.second;

    // TODO: change this to update each module...?
    // Get the time elapsed since the last iteration
    ros::Duration iteration_time = current_time - hebiros_group->prev_time;
    hebiros_group->prev_time = current_time;
    if (hebiros_group->group_added) {
      hebiros_group->UpdateFeedback(iteration_time);
    }
  }
}

//Service callback which adds a group with corresponding joints
bool HebirosGazeboPlugin::SrvAddGroup(AddGroupFromNamesSrv::Request &req,
  AddGroupFromNamesSrv::Response &res) {

  if (hebiros_groups.find(req.group_name) != hebiros_groups.end()) {
    ROS_WARN("Group %s already exists", req.group_name.c_str());
    return true;
  }

  std::shared_ptr<HebirosGazeboGroup> hebiros_group =
    std::make_shared<HebirosGazeboGroup>(req.group_name, this->n);

  hebiros_groups[req.group_name] = hebiros_group;

  for (int i = 0; i < req.families.size(); i++) {
    for (int j = 0; j < req.names.size(); j++) {

      if ((req.families.size() == 1) ||
        (req.families.size() == req.names.size() && i == j)) {

        std::string joint_name = req.families[i]+"/"+req.names[j];
        hebiros_group->feedback.name.push_back(joint_name);

        AddJointToGroup(hebiros_group, req.families[i], req.names[j]);
      }
    }
  }

  int size = hebiros_group->size();

  hebiros_group->feedback.position.resize(size);
  hebiros_group->feedback.motor_winding_temperature.resize(size);
  hebiros_group->feedback.motor_housing_temperature.resize(size);
  hebiros_group->feedback.board_temperature.resize(size);
  hebiros_group->feedback.velocity.resize(size);
  hebiros_group->feedback.effort.resize(size);
  // Default, return "nan" for feedback, until we set something!
  hebiros_group->feedback.position_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  hebiros_group->feedback.velocity_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  hebiros_group->feedback.effort_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  hebiros_group->feedback.accelerometer.resize(size);
  hebiros_group->feedback.gyro.resize(size);

  hebiros_group->feedback_pub = this->n->advertise<FeedbackMsg>(
    "hebiros_gazebo_plugin/feedback/"+req.group_name, 100);

  hebiros_group->group_added = true;

  return true;
}

//Add a joint to an associated group
void HebirosGazeboPlugin::AddJointToGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  const std::string& family, const std::string& name) {

  // Get a weak reference to store in the individual groups
  auto raw_joint = getJoint(family, name);

  // Temporarily, we store joint subscriptions in the gazebo ros plugin here, since
  // the IMU that generates this data is a separate ROS plugin communicating via ROS
  // messages.
  //
  // This will be abstracted into the ROS plugin wrapper in a subsequent refactor
  hebiros_joint_imu_subs.push_back(n->subscribe<sensor_msgs::Imu>(
    "hebiros_gazebo_plugin/imu/" + family + "/" + name, 100, 
    [raw_joint](const boost::shared_ptr<sensor_msgs::Imu const> data) {
      auto a = data->linear_acceleration;
      auto g = data->angular_velocity;
      raw_joint->updateImu(
          {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)},
          {static_cast<float>(g.x), static_cast<float>(g.y), static_cast<float>(g.z)});
    }));

  hebiros_group->AddJoint(raw_joint);

}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
