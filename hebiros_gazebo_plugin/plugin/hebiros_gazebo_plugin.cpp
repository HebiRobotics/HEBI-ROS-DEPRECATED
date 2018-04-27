
#include <hebiros_gazebo_plugin.h>


HebirosGazeboPlugin::HebirosGazeboPlugin() {}

HebirosGazeboPlugin::~HebirosGazeboPlugin() {}

//Load the model and sdf from Gazebo
void HebirosGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
std::cout << "load 3" << std::endl;
  this->model = _model;

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "hebiros_gazebo_plugin_node");
  this->n.reset(new ros::NodeHandle);

  this->add_group_srv =
    this->n->advertiseService<AddGroupFromNamesSrv::Request, AddGroupFromNamesSrv::Response>(
    "/hebiros_gazebo_plugin/add_group", boost::bind(
    &HebirosGazeboPlugin::SrvAddGroup, this, _1, _2));

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
    UpdateGroup(group_pair.second);
  }
}

//Publish feedback and compute PID control to command a joint
void HebirosGazeboPlugin::UpdateGroup(std::shared_ptr<HebirosGazeboGroup> hebiros_group) {

  for (auto joint_pair : hebiros_group->joints) {
    std::string joint_name = joint_pair.first;
    std::shared_ptr<HebirosGazeboJoint> hebiros_joint = hebiros_joints[joint_name];

    physics::JointPtr joint = this->model->GetJoint(joint_name+"/"+hebiros_joint->model_name);

    if (joint) {

      std::shared_ptr<HebirosGazeboJoint> hebiros_joint = joint_pair.second;

      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - hebiros_group->start_time;
      ros::Duration feedback_time = current_time - hebiros_group->prev_feedback_time;

      int i = hebiros_joint->feedback_index;

      double position = joint->GetAngle(0).Radian();
      double velocity = joint->GetVelocity(0);
      physics::JointWrench wrench = joint->GetForceTorque(0);
      double effort = wrench.body1Force.z;

      hebiros_group->feedback.position[i] = position;
      hebiros_group->feedback.velocity[i] = velocity;
      hebiros_group->feedback.effort[i] = effort;

      hebiros_group->feedback.accelerometer[i] = hebiros_joint->accelerometer;
      hebiros_group->feedback.gyro[i] = hebiros_joint->gyro;

      if (hebiros_group->command_received) {
        double force = HebirosGazeboController::ComputeForce(hebiros_group, hebiros_joint,
          position, velocity, effort);

        if ((this->command_lifetime == 0) || (
          //elapsed_time.toSec() <= hebiros_group->command_lifetime/1000.0)) {
          elapsed_time.toSec() <= this->command_lifetime/1000.0)) {

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
        feedback_time.toSec() >= 1.0/this->feedback_frequency) {

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
  hebiros_group->feedback.velocity.resize(size);
  hebiros_group->feedback.effort.resize(size);
  hebiros_group->feedback.position_command.resize(size);
  hebiros_group->feedback.velocity_command.resize(size);
  hebiros_group->feedback.effort_command.resize(size);
  hebiros_group->feedback.accelerometer.resize(size);
  hebiros_group->feedback.gyro.resize(size);

  hebiros_group->feedback_pub = this->n->advertise<FeedbackMsg>(
    "/hebiros_gazebo_plugin/feedback/"+req.group_name, 100);

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

  HebirosGazeboController::SetSettings(hebiros_group, hebiros_joint);
  HebirosGazeboController::ChangeSettings(hebiros_group, hebiros_joint);
}

//Tell Gazebo about this plugin
GZ_REGISTER_MODEL_PLUGIN(HebirosGazeboPlugin);
