#include "hebiros_gazebo_group.h"

HebirosGazeboGroup::HebirosGazeboGroup(std::string name,
  const std::vector<hebi::sim::Joint*>& joints_,
  std::shared_ptr<ros::NodeHandle> n) {

  int size = joints_.size();

  feedback.name.resize(size);
  for (auto joint : joints_)
  {
    joints.push_back(joint);
    feedback.name.push_back(joint->getName());
  }

  feedback.position.resize(size);
  feedback.motor_winding_temperature.resize(size);
  feedback.motor_housing_temperature.resize(size);
  feedback.board_temperature.resize(size);
  feedback.velocity.resize(size);
  feedback.effort.resize(size);
  // Default, return "nan" for feedback, until we set something!
  feedback.position_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  feedback.velocity_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  feedback.effort_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  feedback.accelerometer.resize(size);
  feedback.gyro.resize(size);

  feedback_pub = n->advertise<FeedbackMsg>(
    "hebiros_gazebo_plugin/feedback/" + name, 100);

  this->name = name;

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;
  this->prev_feedback_time = current_time;

  this->command_sub = n->subscribe<CommandMsg>("hebiros_gazebo_plugin/command/"+name, 100,
    boost::bind(&HebirosGazeboGroup::SubCommand, this, _1));

  this->acknowledge_srv =
    n->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    "hebiros_gazebo_plugin/acknowledge/"+name, boost::bind(
    &HebirosGazeboGroup::SrvAcknowledge, this, _1, _2));

  this->command_lifetime_srv =
    n->advertiseService<SetCommandLifetimeSrv::Request, SetCommandLifetimeSrv::Response>(
    "hebiros_gazebo_plugin/set_command_lifetime/"+name, boost::bind(
    &HebirosGazeboGroup::SrvSetCommandLifetime, this, _1, _2));

  this->feedback_frequency_srv =
    n->advertiseService<SetFeedbackFrequencySrv::Request, SetFeedbackFrequencySrv::Response>(
    "hebiros_gazebo_plugin/set_feedback_frequency/"+name, boost::bind(
    &HebirosGazeboGroup::SrvSetFeedbackFrequency, this, _1, _2));
}
  
void HebirosGazeboGroup::UpdateFeedback(const ros::Duration& iteration_time) {
  int i = 0;
  for (auto joint : joints) {

    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - start_time;
    ros::Duration feedback_time = current_time - prev_feedback_time;

    //joint->SetProvideFeedback(true);
    //double velocity = joint->GetVelocity(0);

    feedback.position[i] = joint->getPositionFbk();
    feedback.velocity[i] = joint->getVelocityFbk();
    feedback.effort[i] = joint->getEffortFbk();

    const auto& accel = joint->getAccelerometer();
    feedback.accelerometer[i].x = accel.x();
    feedback.accelerometer[i].y = accel.y();
    feedback.accelerometer[i].z = accel.z();
    const auto& gyro = joint->getGyro();
    feedback.gyro[i].x = gyro.x();
    feedback.gyro[i].y = gyro.y();
    feedback.gyro[i].z = gyro.z();

    // Add temperature feedback
    feedback.motor_winding_temperature[i] = joint->getTemperature().getMotorWindingTemperature();
    feedback.motor_housing_temperature[i] = joint->getTemperature().getMotorHousingTemperature();
    feedback.board_temperature[i] = joint->getTemperature().getActuatorBodyTemperature();

    // Command feedback
    feedback.position_command[i] = joint->getPositionCmd();
    feedback.velocity_command[i] = joint->getVelocityCmd();
    feedback.effort_command[i] = joint->getEffortCmd();

    if (!feedback_pub.getTopic().empty() &&
      feedback_time.toSec() >= 1.0/feedback_frequency) {

      feedback_pub.publish(feedback);
      prev_feedback_time = current_time;
    }

    ++i;
  }
}

// Update the gains for the fields which have information; leave the others unchanged.
// \return `true` if gains changed, `false` otherwise.
bool updateGains(hebi::sim::PidGains& gains, const hebiros::PidGainsMsg& msg, size_t i) {
  bool changed = false;
  if (i < msg.kp.size()) {
    gains.kp_ = msg.kp[i];
    changed = true;
  }
  if (i < msg.ki.size()) {
    gains.ki_ = msg.ki[i];
    changed = true;
  }
  if (i < msg.kd.size()) {
    gains.kd_ = msg.kd[i];
    changed = true;
  }
  if (i < msg.feed_forward.size()) {
    gains.feed_forward_ = msg.feed_forward[i];
    changed = true;
  }
  return changed;
}

void HebirosGazeboGroup::SubCommand(const boost::shared_ptr<CommandMsg const> data) {

  if (this->check_acknowledgement) {
    this->acknowledgement = true;
    this->check_acknowledgement = false;
  }

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;
  ros::Duration elapsed_time = current_time - start_time;

  for (int i = 0; i < data->name.size(); i++) {
    std::string joint_name = data->name[i];

    auto joint_it = std::find_if(joints.begin(), joints.end(), [&joint_name](auto j) { return j->getName() == joint_name; } );

    if (joint_it != joints.end()) {
      auto joint = *joint_it;

      // TODO: SENDER ID!!!!! Generate this properly.
      uint64_t sender_id = 1;

      // Process motion control commands (if any)
      auto p_cmd = std::numeric_limits<double>::quiet_NaN();
      auto v_cmd = std::numeric_limits<double>::quiet_NaN();
      auto e_cmd = std::numeric_limits<double>::quiet_NaN();
      bool has_command = false;
      if (i < data->position.size()) {
        p_cmd = data->position[i];
        has_command = true;
      }
      if (i < data->velocity.size()) {
        v_cmd = data->velocity[i];
        has_command = true;
      }
      if (i < data->effort.size()) {
        e_cmd = data->effort[i];
        has_command = true;
      }
      // TODO: verify nans are handled correctly; what about empty commands?
      if (has_command) {
        joint->setCommand(p_cmd, v_cmd, e_cmd, sender_id, command_lifetime/1000.0, current_time.toSec());
      }

      // Set name
      if (i < data->settings.name.size()) {
        joint->setName(data->settings.name[i]);
      }

      // Change control strategy
      if (i < data->settings.control_strategy.size()) {
        joint->setControlStrategy(static_cast<hebi::sim::Joint::ControlStrategy>(data->settings.control_strategy[i]));
      }

      // TODO: consider changing this to either direct setting of individual parameters,
      // or an entire separate "optional" message layer...

      // Change gains:
      auto current_pos_gains = joint->getPositionPid().getGains();
      if (updateGains(current_pos_gains, data->settings.position_gains, i))
        joint->getPositionPid().setGains(current_pos_gains);

      auto current_vel_gains = joint->getVelocityPid().getGains();
      if (updateGains(current_vel_gains, data->settings.velocity_gains, i))
        joint->getVelocityPid().setGains(current_vel_gains);

      auto current_eff_gains = joint->getEffortPid().getGains();
      if (updateGains(current_eff_gains, data->settings.effort_gains, i))
        joint->getEffortPid().setGains(current_eff_gains);

    }
  }
}

//Service callback which acknowledges that a command has been received
bool HebirosGazeboGroup::SrvAcknowledge(std_srvs::Empty::Request &req,
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

bool HebirosGazeboGroup::SrvSetCommandLifetime(SetCommandLifetimeSrv::Request &req,
  SetCommandLifetimeSrv::Response &res) {

  command_lifetime = req.command_lifetime;
  return true;
}

bool HebirosGazeboGroup::SrvSetFeedbackFrequency(SetFeedbackFrequencySrv::Request &req,
  SetFeedbackFrequencySrv::Response &res) {

  feedback_frequency = req.feedback_frequency;
  return true;
}






