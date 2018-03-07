
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

  this->command_sub = 
    this->n->subscribe<CommandMsg>("/hebiros_gazebo_plugin/command", 100,
    boost::bind(&HebirosGazeboPlugin::SubCommand, this, _1));

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

  for (auto joint_pair : hebiros_joints) {
    std::string joint_name = joint_pair.first;
    std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_joints[joint_name];

    physics::JointPtr joint = this->model->GetJoint(joint_name+"/"+hebiros_joint->model_name);

    if (joint) {
      UpdateJoint(joint_name, joint);
    }
    else {
      ROS_WARN("Joint %s not found", joint_name.c_str());
    }
  }
}

//Publish feedback and compute PID control to command a joint
void HebirosGazeboPlugin::UpdateJoint(
  std::string joint_name, physics::JointPtr joint) {

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_joints[joint_name];

  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - hebiros_joint->start_time;
  ros::Duration feedback_time = current_time - hebiros_joint->prev_feedback_time;

  double position = joint->GetAngle(0).Radian();
  double velocity = joint->GetVelocity(0);
  physics::JointWrench wrench = joint->GetForceTorque(0);
  double effort = wrench.body1Force.z;

  hebiros_joint->feedback.name = {joint_name};
  hebiros_joint->feedback.position = {position};
  hebiros_joint->feedback.velocity = {velocity};
  hebiros_joint->feedback.effort = {effort};

  if (hebiros_joint->command_received) {
    double force = this->controller.ComputeForce(hebiros_joint, position, velocity, effort);

    if ((this->command_lifetime == 0) || (
      elapsed_time.toSec() <= this->command_lifetime/1000.0)) {

      joint->SetForce(0, force);
    }

    int i = hebiros_joint->command_index;
    if (i < hebiros_joint->command_target.position.size()) {
      hebiros_joint->feedback.position_command = {hebiros_joint->command_target.position[i]};
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

    hebiros_joint->feedback_publisher.publish(hebiros_joint->feedback);
    hebiros_joint->prev_feedback_time = current_time;
  }
}

//Subscriber callback which receives a group set of commands
void HebirosGazeboPlugin::SubCommand(const boost::shared_ptr<CommandMsg const> data) {

  if (this->check_acknowledgement) {
    this->acknowledgement = true;
    this->check_acknowledgement = false;
  }

  for (int i = 0; i < data->name.size(); i++) {
    std::string joint_name = data->name[i];

    if (hebiros_joints.find(joint_name) == hebiros_joints.end()) {
      AddJoint(joint_name);
    }

    std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_joints[joint_name];
    hebiros_joint->Reset(i, *data);
    hebiros_joint->settings.name = {data->name[i]};
    this->controller.ChangeSettings(hebiros_joint);
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

  std::cout << "group " << req.group_name << std::endl;

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

//Add a joint to the list of joints if it has not yet been commanded by name
void HebirosGazeboPlugin::AddJoint(std::string joint_name) {

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint =
    std::make_shared<HebirosGazeboJoint>(joint_name, this->n);
  hebiros_joints[joint_name] = hebiros_joint;

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

  hebiros_joint->feedback_publisher = this->n->advertise<FeedbackMsg>(
    "/hebiros_gazebo_plugin/feedback/"+joint_name, 100);

  this->controller.SetSettings(hebiros_joint);
  this->controller.ChangeSettings(hebiros_joint);
}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
