
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

  this->acknowledge_srv =
    this->n->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    "/hebiros_gazebo_plugin/acknowledge", boost::bind(
    &HebirosGazeboPlugin::SrvAcknowledge, this, _1, _2));

  this->command_lifetime_srv =
    this->n->advertiseService<SetCommandLifetimeSrv::Request,
    SetCommandLifetimeSrv::Response>("/hebiros_gazebo_plugin/set_command_lifetime",
    boost::bind(&HebirosGazeboPlugin::SrvSetCommandLifetime, this, _1, _2));

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

  if ((this->command_lifetime != 0) && (
    elapsed_time.toSec() > this->command_lifetime/1000.0)) {
    
    return;
  }

  double position = joint->GetAngle(0).Radian();
  double velocity = joint->GetVelocity(0);
  physics::JointWrench wrench = joint->GetForceTorque(0);
  double effort = wrench.body1Force.z;

  sensor_msgs::JointState feedback_msg;
  feedback_msg.name = {joint_name};
  feedback_msg.position = {position};
  feedback_msg.velocity = {velocity};
  feedback_msg.effort = {effort};

  if (!hebiros_joint->publisher.getTopic().empty()) {
    hebiros_joint->publisher.publish(feedback_msg);
  }

  if (hebiros_joint->command_received) {
    double force = this->controller.ComputeForce(hebiros_joint, position, velocity, effort);
    joint->SetForce(0, force);
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

//Service callback which sets the command lifetime for all joints
bool HebirosGazeboPlugin::SrvSetCommandLifetime(SetCommandLifetimeSrv::Request &req,
  SetCommandLifetimeSrv::Response &res) {

  this->command_lifetime = req.command_lifetime;

  return true;
}

//Add a joint to the list of joints if it has not yet been commanded by name
void HebirosGazeboPlugin::AddJoint(std::string joint_name) {

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint =
    std::make_shared<HebirosGazeboJoint>(joint_name);
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

  hebiros_joint->publisher = this->n->advertise<sensor_msgs::JointState>(
    "/hebiros_gazebo_plugin/feedback/"+joint_name, 100);

  this->controller.SetSettings(hebiros_joint);
  this->controller.ChangeSettings(hebiros_joint);
}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
