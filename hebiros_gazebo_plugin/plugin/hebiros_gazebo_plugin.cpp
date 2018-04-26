
#include <hebiros_gazebo_plugin.h>


HebirosGazeboPlugin::HebirosGazeboPlugin() {}

HebirosGazeboPlugin::~HebirosGazeboPlugin() {}

//Load the model and sdf from Gazebo
void HebirosGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  this->model = _model;

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "hebiros_gazebo_plugin_node");
  this->n.reset(new ros::NodeHandle);

  this->check_acknowledgement = false;
  this->acknowledgement = false;

  this->add_group_srv =
    this->n->advertiseService<AddGroupFromNamesSrv::Request, AddGroupFromNamesSrv::Response>(
    "/hebiros_gazebo_plugin/add_group", boost::bind(
    &HebirosGazeboPlugin::SrvAddGroup, this, _1, _2));

  this->acknowledge_srv =
    this->n->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    "/hebiros_gazebo_plugin/acknowledge", boost::bind(
    &HebirosGazeboPlugin::SrvAcknowledge, this, _1, _2));

  this->command_lifetime_srv =
    this->n->advertiseService<SetCommandLifetimeSrv::Request,
    SetCommandLifetimeSrv::Response>("/hebiros_gazebo_plugin/set_command_lifetime",
    boost::bind(&HebirosGazeboPlugin::SrvSetCommandLifetime, this, _1, _2));

  this->feedback_frequency_srv =
    this->n->advertiseService<SetFeedbackFrequencySrv::Request,
    SetFeedbackFrequencySrv::Response>("/hebiros_gazebo_plugin/set_feedback_frequency",
    boost::bind(&HebirosGazeboPlugin::SrvSetFeedbackFrequency, this, _1, _2));

  this->update_connection = event::Events::ConnectWorldUpdateBegin (
    boost::bind(&HebirosGazeboPlugin::OnUpdate, this, _1));

  ROS_INFO("Loaded hebiros gazebo plugin");
}

//Update the joints at every simulation iteration
void HebirosGazeboPlugin::OnUpdate(const common::UpdateInfo & _info) {

  for (auto group_pair : hebiros_groups) {
    group_pair.second
  }
}

//Publish feedback and compute PID control to command a joint
void HebirosGazeboPlugin::UpdateGroup(
  std::string joint_name, physics::JointPtr joint) {

  for (auto joint_pair : hebiros_group.joints) {
    std::string joint_name = joint_pair.first;
    std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_joints[joint_name];

    physics::JointPtr joint = this->model->GetJoint(joint_name+"/"+hebiros_joint->model_name);

    if (joint) {

      std::shared_ptr<HebirosGazeboJoint> hebiros_joint = joint_pair.second;

      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - hebiros_joint->start_time;
      ros::Duration feedback_time = current_time - hebiros_joint->prev_feedback_time;

      int i = hebiros_joint->feedback_index;

      double position = joint->GetAngle(0).Radian();
      double velocity = joint->GetVelocity(0);
      physics::JointWrench wrench = joint->GetForceTorque(0);
      double effort = wrench.body1Force.z;

      hebiros_group->feedback_msg.position[i] = position;
      hebiros_group->feedback_msg.velocity[i] = velocity;
      hebiros_group->feedback_msg.effort[i] = effort;

      hebiros_group->feedback_msg.accelerometer[i] = hebiros_joint.accelerometer;
      hebiros_group->feedback_msg.gyro[i] = hebiros_joint.gyro;

      if (hebiros_joint->command_received) {
        double force = this->controller.ComputeForce(hebiros_joint, position, velocity, effort);

        if ((this->command_lifetime == 0) || (
          elapsed_time.toSec() <= hebiros_group->command_lifetime/1000.0)) {

          joint->SetForce(0, force);
        }

        int j = hebiros_joint->command_index;
        if (i < hebiros_joint->command_target.position.size()) {
          hebiros_group->feedback_msg.position_command[j] = hebiros_joint->command_target.position[i];
        }
        if (i < hebiros_joint->command_target.velocity.size()) {
          hebiros_joint->feedback.velocity_command = {hebiros_joint->command_target.velocity[i]};
        }
        if (i < hebiros_joint->command_target.effort.size()) {
          hebiros_joint->feedback.effort_command = {hebiros_joint->command_target.effort[i]};
        }
      }

      if (!hebiros_joint->feedback_publisher.getTopic().empty() &&
        feedback_time.toSec() >= 1.0/this->feedback_frequency) {

        hebiros_group->feedback_publisher.publish(hebiros_group->feedback_msg);
        hebiros_group->prev_feedback_time = current_time;
      }
    }
    else {
      ROS_WARN("Joint %s not found", joint_name.c_str());
    }
  }
}

//Service callback which acknowledges that a command has been received
bool HebirosGazeboPlugin::SrvAcknowledge(std_srvs::Empty::Request &req,
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

//Service callback which adds a group with corresponding joints
bool HebirosGazeboPlugin::SrvAddGroup(AddGroupFromNamesSrv::Request &req,
  AddGroupFromNamesSrv::Response &res) {

  std::shared_ptr<HebirosGazeboGroup> hebiros_group =
    std::make_shared<HebirosGazeboGroup>(req.group_name, this->n);
  hebiros_groups[req.group_name] = hebiros_group;

  for (int i = 0; i < req.families.size(); i++) {
    for (int j = 0; j < req.names.size(); j++) {

      if ((req.families.size() == 1) ||
        (req.families.size() == req.names.size() && i == j)) {

        std::string joint_name = req.families[i]+"/"+req.names[j];
        hebiros_group->feedback_msg.name.push_back(joint_name);

        AddJointToGroup(hebiros_group, joint_name);
      }
    }
  }

  int size = hebiros_group->joints.size();

  hebiros_group->feedback_msg.position.resize(size);
  hebiros_group->feedback_msg.velocity.resize(size);
  hebiros_group->feedback_msg.effort.resize(size);
  hebiros_group->feedback_msg.position_command.resize(size);
  hebiros_group->feedback_msg.velocity_command.resize(size);
  hebiros_group->feedback_msg.effort_command.resize(size);
  hebiros_group->feedback_msg.accelerometer.resize(size);
  hebiros_group->feedback_msg.gyro.resize(size);

  hebiros_group->feedback_publisher = this->n->advertise<FeedbackMsg>(
    "/hebiros_gazebo_plugin/feedback/"+group_name, 100);

  return true;
}

//Service callback which sets the command lifetime for all joints
bool HebirosGazeboPlugin::SrvSetCommandLifetime(SetCommandLifetimeSrv::Request &req,
  SetCommandLifetimeSrv::Response &res) {

  this->command_lifetime = req.command_lifetime;

  return true;
}

//Service callback which sets the feedback frequency for all joints
bool HebirosGazeboPlugin::SrvSetFeedbackFrequency(SetFeedbackFrequencySrv::Request &req,
  SetFeedbackFrequencySrv::Response &res) {

  this->feedback_frequency = req.feedback_frequency;

  return true;
}

//Add a joint to an associated group
void HebirosGazeboPlugin::AddJointToGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::string joint_name) {

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint =
    std::make_shared<HebirosGazeboJoint>(joint_name, this->n);

  hebiros_joint->feedback_index = hebiros_group->joints.size();
  hebiros_joint->command_index = hebiros_joint->feedback_index;
  hebiros_group->joints[joint_name] = hebiros_joint;

  physics::JointPtr joint;
  if (joint = this->model->GetJoint(joint_name+"/X5_1")) {
    hebiros_joint->model_name = "X5_1";
  }
  else if (joint = this->model->GetJoint(joint_name+"/X5_4")) {
    hebiros_joint->model_name = "X5_4";
  }
  else if (joint = this->model->GetJoint(joint_name+"/X5_9")) {
    hebiros_joint->model_name = "X5_9";
  }

  this->controller.SetSettings(hebiros_group, hebiros_joint);
  this->controller.ChangeSettings(hebiros_group, hebiros_joint);
}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
