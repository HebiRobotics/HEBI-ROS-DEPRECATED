
#include <hebiros_gazebo_plugin.h>


HebirosGazeboPlugin::HebirosGazeboPlugin() {}

HebirosGazeboPlugin::~HebirosGazeboPlugin() {}

void HebirosGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

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

  this->model = _model;

  this->update_connection = event::Events::ConnectWorldUpdateBegin (
    boost::bind(&HebirosGazeboPlugin::OnUpdate, this, _1));

  ROS_INFO("Loaded hebiros gazebo plugin");
}

void HebirosGazeboPlugin::OnUpdate(const common::UpdateInfo & _info) {

  for (auto joint_pair : hebiros_joints) {
    std::string joint_name = joint_pair.first;

    physics::JointPtr joint = this->model->GetJoint(joint_name);
    if (joint) {
      UpdateJoint(joint_name, joint);
    }
    else {
      ROS_WARN("Joint %s not found", joint_name.c_str());
    }
  }
}

void HebirosGazeboPlugin::UpdateJoint(
  std::string joint_name, physics::JointPtr joint) {

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_joints[joint_name];
  int i = hebiros_joint->command_index;
  CommandMsg target = hebiros_joint->command_target;

  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - hebiros_joint->start_time;
  ros::Duration iteration_time = current_time - hebiros_joint->prev_time;
  hebiros_joint->prev_time = current_time;

  if (elapsed_time.toSec() > this->command_lifetime) {
    return;
  }

  double target_position, target_velocity, target_effort;
  double position_error_p, position_error_i, position_error_d;
  double velocity_error_p, velocity_error_i, velocity_error_d;
  double effort_error_p, effort_error_i, effort_error_d;
  double position_force, velocity_force, effort_force;
  double force;

  double position = joint->GetAngle(0).Radian();
  double velocity = joint->GetVelocity(0);
  double effort = joint->GetForce(0);

  sensor_msgs::JointState feedback_msg;
  feedback_msg.name = {joint_name};
  feedback_msg.position = {position};
  feedback_msg.velocity = {velocity};
  feedback_msg.effort = {effort};
  hebiros_joint->publisher.publish(feedback_msg);

  if (i < target.position.size()) {
    target_position = target.position[i];
  }
  else {
    target_position = position;
  }
  if (i < target.velocity.size()) {
    target_velocity = target.velocity[i];
  }
  else {
    target_velocity = velocity;
  }
  if (i < target.effort.size()) {
    target_effort = target.effort[i];
  }
  else {
    target_effort = effort;
  }

  if (i < target.settings.position_gains.kp.size()) {
    hebiros_joint->settings.position_gains.kp = {target.settings.position_gains.kp[i]};
  }
  if (i < target.settings.position_gains.ki.size()) {
    hebiros_joint->settings.position_gains.ki = {target.settings.position_gains.ki[i]};
  }
  if (i < target.settings.position_gains.kd.size()) {
    hebiros_joint->settings.position_gains.kd = {target.settings.position_gains.kd[i]};
  }

  if (i < target.settings.velocity_gains.kp.size()) {
    hebiros_joint->settings.velocity_gains.kp = {target.settings.velocity_gains.kp[i]};
  }
  if (i < target.settings.velocity_gains.ki.size()) {
    hebiros_joint->settings.velocity_gains.ki = {target.settings.velocity_gains.ki[i]};
  }
  if (i < target.settings.velocity_gains.kd.size()) {
    hebiros_joint->settings.velocity_gains.kd = {target.settings.velocity_gains.kd[i]};
  }

  if (i < target.settings.effort_gains.kp.size()) {
    hebiros_joint->settings.effort_gains.kp = {target.settings.effort_gains.kp[i]};
  }
  if (i < target.settings.effort_gains.ki.size()) {
    hebiros_joint->settings.effort_gains.ki = {target.settings.effort_gains.ki[i]};
  }
  if (i < target.settings.effort_gains.kd.size()) {
    hebiros_joint->settings.effort_gains.kd = {target.settings.effort_gains.kd[i]};
  }

  position_error_p = target_position - position;
  position_error_i = hebiros_joint->position_elapsed_error + position_error_p;
  position_error_d = (position_error_p - hebiros_joint->position_prev_error) /
    iteration_time.toSec();
  hebiros_joint->position_prev_error = position_error_p;
  hebiros_joint->position_elapsed_error = position_error_i;

  velocity_error_p = target_velocity - velocity;
  velocity_error_i = hebiros_joint->velocity_elapsed_error + velocity_error_p;
  velocity_error_d = (velocity_error_p - hebiros_joint->velocity_prev_error) /
    iteration_time.toSec();
  hebiros_joint->velocity_prev_error = velocity_error_p;
  hebiros_joint->velocity_elapsed_error = velocity_error_i;

  effort_error_p = target_effort - effort;
  effort_error_i = hebiros_joint->effort_elapsed_error + effort_error_p;
  effort_error_d = (effort_error_p - hebiros_joint->effort_prev_error) /
    iteration_time.toSec();
  hebiros_joint->effort_prev_error = effort_error_p;
  hebiros_joint->effort_elapsed_error = effort_error_i;

  if (iteration_time.toSec() <= 0) {
    position_error_d = 0;
    velocity_error_d = 0;
    effort_error_d = 0;
  }

  position_force = (hebiros_joint->settings.position_gains.kp[0] * position_error_p) +
    (hebiros_joint->settings.position_gains.ki[0] * position_error_i) +
    (hebiros_joint->settings.position_gains.kd[0] * position_error_d);

  velocity_force = (hebiros_joint->settings.velocity_gains.kp[0] * velocity_error_p) +
    (hebiros_joint->settings.velocity_gains.ki[0] * velocity_error_i) +
    (hebiros_joint->settings.velocity_gains.kd[0] * velocity_error_d);

  effort_force = (hebiros_joint->settings.effort_gains.kp[0] * effort_error_p) +
    (hebiros_joint->settings.effort_gains.ki[0] * effort_error_i) +
    (hebiros_joint->settings.effort_gains.kd[0] * effort_error_d);

  force = position_force + velocity_force + effort_force;
  //double output = ((force - ((velocity*762.22)/1530)) / 9.99) * (6.26/1000);

  joint->SetForce(0, force);
}

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
  }

}

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

void HebirosGazeboPlugin::AddJoint(std::string joint_name) {

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint =
    std::make_shared<HebirosGazeboJoint>(joint_name);
  hebiros_joints[joint_name] = hebiros_joint;

  hebiros_joint->settings.position_gains.kp = {this->DEFAULT_POSITION_KP};
  hebiros_joint->settings.position_gains.ki = {this->DEFAULT_POSITION_KI};
  hebiros_joint->settings.position_gains.kd = {this->DEFAULT_POSITION_KD};
  hebiros_joint->settings.velocity_gains.kp = {this->DEFAULT_VELOCITY_KP};
  hebiros_joint->settings.velocity_gains.ki = {this->DEFAULT_VELOCITY_KI};
  hebiros_joint->settings.velocity_gains.kd = {this->DEFAULT_VELOCITY_KD};
  hebiros_joint->settings.effort_gains.kp = {this->DEFAULT_EFFORT_KP};
  hebiros_joint->settings.effort_gains.ki = {this->DEFAULT_EFFORT_KI};
  hebiros_joint->settings.effort_gains.kd = {this->DEFAULT_EFFORT_KD};

  hebiros_joint->publisher = this->n->advertise<sensor_msgs::JointState>(
    "/hebiros_gazebo_plugin/feedback/"+joint_name, 100);
}


GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
