#include <hebiros_gazebo_plugin.h>

//Load the model and sdf from Gazebo
void HebirosGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->model = _model;

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "hebiros_gazebo_plugin_node");

  this->add_group_srv =
    this->n->advertiseService<AddGroupFromNamesSrv::Request, AddGroupFromNamesSrv::Response>(
    "hebiros_gazebo_plugin/add_group", boost::bind(
    &HebirosGazeboPlugin::SrvAddGroup, this, _1, _2));

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
void HebirosGazeboPlugin::OnUpdate(const common::UpdateInfo & _info) {
  ros::Time current_time = ros::Time::now();

  for (auto group_pair : hebiros_groups) {
    auto hebiros_group = group_pair.second;

    // Get the time elapsed since the last iteration
    ros::Duration iteration_time = current_time - hebiros_group->prev_time;
    hebiros_group->prev_time = current_time;
    if (hebiros_group->group_added) {
      UpdateGroup(hebiros_group, iteration_time);
    }
  }
}

//Publish feedback and compute PID control to command a joint
void HebirosGazeboPlugin::UpdateGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group, const ros::Duration& iteration_time) {
  for (auto joint_pair : hebiros_group->joints) {

    std::string joint_name = joint_pair.first;
    std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_group->joints[joint_name];

    physics::JointPtr joint = this->model->GetJoint(joint_name+"/"+hebiros_joint->model_name);

    if (joint) {

      std::shared_ptr<HebirosGazeboJoint> hebiros_joint = joint_pair.second;

      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - hebiros_group->start_time;
      ros::Duration feedback_time = current_time - hebiros_group->prev_feedback_time;

      int i = hebiros_joint->feedback_index;

      joint->SetProvideFeedback(true);
      double position = joint->GetAngle(0).Radian();
      double velocity = joint->GetVelocity(0);
      physics::JointWrench wrench = joint->GetForceTorque(0);
      auto trans = joint->GetChild()->GetInitialRelativePose().rot;
      double effort = (-1 * (trans * wrench.body1Torque)).z;

      hebiros_group->feedback.position[i] = position;
      hebiros_group->feedback.velocity[i] = velocity;
      hebiros_group->feedback.effort[i] = effort;

      hebiros_group->feedback.accelerometer[i] = hebiros_joint->accelerometer;
      hebiros_group->feedback.gyro[i] = hebiros_joint->gyro;

      // Add temperature feedback
      hebiros_group->feedback.motor_winding_temperature[i] = hebiros_joint->temperature.getMotorWindingTemperature();
      hebiros_group->feedback.motor_housing_temperature[i] = hebiros_joint->temperature.getMotorHousingTemperature();
      hebiros_group->feedback.board_temperature[i] = hebiros_joint->temperature.getActuatorBodyTemperature();

      if (hebiros_group->command_received) {
        double force = HebirosGazeboController::ComputeForce(hebiros_group, hebiros_joint,
          position, velocity, effort, iteration_time);

        if ((hebiros_group->command_lifetime == 0) || (
          elapsed_time.toSec() <= hebiros_group->command_lifetime/1000.0)) {

          joint->SetForce(0, force);
        }

        int j = hebiros_joint->command_index;
        if (i < hebiros_group->command_target.position.size()) {
          hebiros_group->feedback.position_command[j] = hebiros_group->command_target.position[j];
        }
        if (i < hebiros_group->command_target.velocity.size()) {
          hebiros_group->feedback.velocity_command[j] = hebiros_group->command_target.velocity[j];
        }
        if (i < hebiros_group->command_target.effort.size()) {
          hebiros_group->feedback.effort_command[j] = hebiros_group->command_target.effort[j];
        }
      }

      if (!hebiros_group->feedback_pub.getTopic().empty() &&
        feedback_time.toSec() >= 1.0/hebiros_group->feedback_frequency) {

        hebiros_group->feedback_pub.publish(hebiros_group->feedback);
        hebiros_group->prev_feedback_time = current_time;
      }
    }
    else {
      ROS_WARN("Joint %s not found", joint_name.c_str());
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
  hebiros_group->feedback.position_command.resize(size);
  hebiros_group->feedback.velocity_command.resize(size);
  hebiros_group->feedback.effort_command.resize(size);
  hebiros_group->feedback.accelerometer.resize(size);
  hebiros_group->feedback.gyro.resize(size);

  hebiros_group->feedback_pub = this->n->advertise<FeedbackMsg>(
    "hebiros_gazebo_plugin/feedback/"+req.group_name, 100);

  hebiros_group->group_added = true;

  return true;
}

//Add a joint to an associated group
void HebirosGazeboPlugin::AddJointToGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::string joint_name) {

  std::string model_name = "";
  bool is_x8 = false;
  if (this->model->GetJoint(joint_name+"/X5_1")) {
    model_name = "X5_1";
  }
  else if (this->model->GetJoint(joint_name+"/X5_4")) {
    model_name = "X5_4";
  }
  else if (this->model->GetJoint(joint_name+"/X5_9")) {
    model_name = "X5_9";
  }
  else if (this->model->GetJoint(joint_name+"/X8_3")) {
    model_name = "X8_3";
    is_x8 = true;
  }
  else if (this->model->GetJoint(joint_name+"/X8_9")) {
    model_name = "X8_9";
    is_x8 = true;
  }
  else if (this->model->GetJoint(joint_name+"/X8_16")) {
    model_name = "X8_16";
    is_x8 = true;
  }

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint =
    std::make_shared<HebirosGazeboJoint>(joint_name, model_name, is_x8, this->n);

  hebiros_joint->feedback_index = hebiros_group->joints.size();
  hebiros_joint->command_index = hebiros_joint->feedback_index;

  HebirosGazeboController::SetSettings(hebiros_group, hebiros_joint);
  hebiros_group->joints[joint_name] = hebiros_joint;
}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
