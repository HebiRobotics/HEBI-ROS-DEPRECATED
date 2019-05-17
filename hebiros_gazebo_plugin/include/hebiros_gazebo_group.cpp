#include <hebiros_gazebo_group.h>

HebirosGazeboGroup::HebirosGazeboGroup(std::string name,
  std::shared_ptr<ros::NodeHandle> n) {
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

    if (joints.find(joint_name) != joints.end()) {
      auto hebiros_joint = joints[joint_name];

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
        hebiros_joint->setCommand(p_cmd, v_cmd, e_cmd, sender_id, command_lifetime/1000.0, current_time.toSec());
      }

      // Set name
      if (i < data->settings.name.size()) {
        hebiros_joint->name = data->settings.name[i];
      }

      // Change control strategy
      if (i < data->settings.control_strategy.size()) {
        hebiros_joint->setControlStrategy(static_cast<hebi::sim::Joint::ControlStrategy>(data->settings.control_strategy[i]));
      }

      // TODO: consider changing this to either direct setting of individual parameters,
      // or an entire separate "optional" message layer...

      // Change gains:
      auto current_pos_gains = hebiros_joint->position_pid.getGains();
      if (updateGains(current_pos_gains, data->settings.position_gains, i))
        hebiros_joint->position_pid.setGains(current_pos_gains);

      auto current_vel_gains = hebiros_joint->velocity_pid.getGains();
      if (updateGains(current_vel_gains, data->settings.velocity_gains, i))
        hebiros_joint->velocity_pid.setGains(current_vel_gains);

      auto current_eff_gains = hebiros_joint->effort_pid.getGains();
      if (updateGains(current_eff_gains, data->settings.effort_gains, i))
        hebiros_joint->effort_pid.setGains(current_eff_gains);

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






