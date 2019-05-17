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
      UpdateGroup(hebiros_group, iteration_time);
    }
  }
}

// Publish feedback and compute PID control to command a joint
// TODO: move this to the group?
void HebirosGazeboPlugin::UpdateGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group, const ros::Duration& iteration_time) {
  for (auto joint_pair : hebiros_group->joints) {

    auto hebiros_joint = joint_pair.second;

    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - hebiros_group->start_time;
    ros::Duration feedback_time = current_time - hebiros_group->prev_feedback_time;

    int i = hebiros_joint->feedback_index;

    //joint->SetProvideFeedback(true);
    //double velocity = joint->GetVelocity(0);

    hebiros_group->feedback.position[i] = hebiros_joint->position_fbk;
    hebiros_group->feedback.velocity[i] = hebiros_joint->velocity_fbk;
    hebiros_group->feedback.effort[i] = hebiros_joint->effort_fbk;

    const auto& accel = hebiros_joint->getAccelerometer();
    hebiros_group->feedback.accelerometer[i].x = accel.x();
    hebiros_group->feedback.accelerometer[i].y = accel.y();
    hebiros_group->feedback.accelerometer[i].z = accel.z();
    const auto& gyro = hebiros_joint->getGyro();
    hebiros_group->feedback.gyro[i].x = gyro.x();
    hebiros_group->feedback.gyro[i].y = gyro.y();
    hebiros_group->feedback.gyro[i].z = gyro.z();

    // Add temperature feedback
    hebiros_group->feedback.motor_winding_temperature[i] = hebiros_joint->temperature.getMotorWindingTemperature();
    hebiros_group->feedback.motor_housing_temperature[i] = hebiros_joint->temperature.getMotorHousingTemperature();
    hebiros_group->feedback.board_temperature[i] = hebiros_joint->temperature.getActuatorBodyTemperature();

    // Command feedback
    hebiros_group->feedback.position_command[i] = hebiros_joint->position_cmd;
    hebiros_group->feedback.velocity_command[i] = hebiros_joint->velocity_cmd;
    hebiros_group->feedback.effort_command[i] = hebiros_joint->effort_cmd;

    if (!hebiros_group->feedback_pub.getTopic().empty() &&
      feedback_time.toSec() >= 1.0/hebiros_group->feedback_frequency) {

      hebiros_group->feedback_pub.publish(hebiros_group->feedback);
      hebiros_group->prev_feedback_time = current_time;
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

        AddJointToGroup(hebiros_group, joint_name);
      }
    }
  }

  int size = hebiros_group->joints.size();

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

void updateImu(const boost::shared_ptr<sensor_msgs::Imu const> data) {
}

//Add a joint to an associated group
void HebirosGazeboPlugin::AddJointToGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::string joint_name) {

  std::string model_name = "";
  bool is_x8 = false;
  if (model_->GetJoint(joint_name+"/X5_1")) {
    model_name = "X5_1";
  }
  else if (model_->GetJoint(joint_name+"/X5_4")) {
    model_name = "X5_4";
  }
  else if (model_->GetJoint(joint_name+"/X5_9")) {
    model_name = "X5_9";
  }
  else if (model_->GetJoint(joint_name+"/X8_3")) {
    model_name = "X8_3";
    is_x8 = true;
  }
  else if (model_->GetJoint(joint_name+"/X8_9")) {
    model_name = "X8_9";
    is_x8 = true;
  }
  else if (model_->GetJoint(joint_name+"/X8_16")) {
    model_name = "X8_16";
    is_x8 = true;
  }

  // Get a weak reference to store in the individual groups
  auto raw_joint = addJoint(std::make_unique<hebi::sim::Joint>(joint_name, model_name, is_x8));

  // Temporarily, we store joint subscriptions in the gazebo ros plugin here, since
  // the IMU that generates this data is a separate ROS plugin communicating via ROS
  // messages.
  //
  // This will be abstracted into the ROS plugin wrapper in a subsequent refactor
  hebiros_joint_imu_subs.push_back(n->subscribe<sensor_msgs::Imu>(
    "hebiros_gazebo_plugin/imu/" + joint_name, 100, 
    [raw_joint](const boost::shared_ptr<sensor_msgs::Imu const> data) {
      auto a = data->linear_acceleration;
      auto g = data->angular_velocity;
      raw_joint->updateImu(
          {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)},
          {static_cast<float>(g.x), static_cast<float>(g.y), static_cast<float>(g.z)});
    }));


  raw_joint->feedback_index = hebiros_group->joints.size();

  // TODO: move to joint constructor...
  HebirosGazeboController::SetDefaultGains(raw_joint);
  hebiros_group->joints[joint_name] = raw_joint;

}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
