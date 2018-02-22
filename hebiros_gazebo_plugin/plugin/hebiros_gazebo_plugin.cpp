
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

void HebirosGazeboPlugin::UpdateJoint(
  std::string joint_name, physics::JointPtr joint) {

  std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_joints[joint_name];

  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - hebiros_joint->start_time;

  if (elapsed_time.toSec() > this->command_lifetime) {
    return;
  }

  double position = joint->GetAngle(0).Radian();
  double velocity = joint->GetVelocity(0);
  double effort = joint->GetForce(0);

  sensor_msgs::JointState feedback_msg;
  feedback_msg.name = {joint_name};
  feedback_msg.position = {position};
  feedback_msg.velocity = {velocity};
  feedback_msg.effort = {effort};
  hebiros_joint->publisher.publish(feedback_msg);

  double force = this->controller.ComputeForce(hebiros_joint, position, velocity, effort);

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
    this->controller.ChangeSettings(hebiros_joint);
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

  physics::JointPtr joint;
  if (joint = this->model->GetJoint(joint_name+"/X5-1")) {
    hebiros_joint->model_name = "X5-1";
  }
  else if (joint = this->model->GetJoint(joint_name+"/X5-4")) {
    hebiros_joint->model_name = "X5-4";
  }
  else if (joint = this->model->GetJoint(joint_name+"/X5-9")) {
    hebiros_joint->model_name = "X5-9";
  }

  hebiros_joint->publisher = this->n->advertise<sensor_msgs::JointState>(
    "/hebiros_gazebo_plugin/feedback/"+joint_name, 100);

  this->controller.SetSettings(hebiros_joint);
  this->controller.ChangeSettings(hebiros_joint);
}


GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
