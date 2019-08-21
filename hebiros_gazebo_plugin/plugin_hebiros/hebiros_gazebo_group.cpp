#include "hebiros_gazebo_group.h"

namespace hebi {
namespace sim {
namespace plugin {

HebirosGazeboGroup::HebirosGazeboGroup(std::string name,
  const std::vector<hebi::sim::Joint*>& joints,
  std::shared_ptr<ros::NodeHandle> n) {

  int size = joints.size();

  feedback_.name.resize(size);
  for (auto joint : joints)
  {
    joints_.push_back(joint);
    feedback_.name.push_back(joint->getName());
  }

  feedback_.position.resize(size);
  feedback_.motor_winding_temperature.resize(size);
  feedback_.motor_housing_temperature.resize(size);
  feedback_.board_temperature.resize(size);
  feedback_.velocity.resize(size);
  feedback_.effort.resize(size);
  // Default, return "nan" for feedback_, until we set something!
  feedback_.position_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  feedback_.velocity_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  feedback_.effort_command.resize(size, std::numeric_limits<float>::quiet_NaN());
  feedback_.accelerometer.resize(size);
  feedback_.gyro.resize(size);

  feedback_pub_ = n->advertise<hebiros::FeedbackMsg>(
    "hebiros_gazebo_plugin/feedback/" + name, 100);

  name_ = name;

  ros::Time current_time = ros::Time::now();
  start_time_ = current_time;
  prev_time_ = current_time;
  prev_feedback_time_ = current_time;

  command_sub_ = n->subscribe<hebiros::CommandMsg>("hebiros_gazebo_plugin/command/"+name, 100,
    boost::bind(&HebirosGazeboGroup::SubCommand, this, _1));

  acknowledge_srv_ =
    n->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    "hebiros_gazebo_plugin/acknowledge/"+name, boost::bind(
    &HebirosGazeboGroup::SrvAcknowledge, this, _1, _2));

  command_lifetime_srv_ =
    n->advertiseService<hebiros::SetCommandLifetimeSrv::Request, hebiros::SetCommandLifetimeSrv::Response>(
    "hebiros_gazebo_plugin/set_command_lifetime/"+name, boost::bind(
    &HebirosGazeboGroup::SrvSetCommandLifetime, this, _1, _2));

  feedback_frequency_srv_ =
    n->advertiseService<hebiros::SetFeedbackFrequencySrv::Request, hebiros::SetFeedbackFrequencySrv::Response>(
    "hebiros_gazebo_plugin/set_feedback_frequency/"+name, boost::bind(
    &HebirosGazeboGroup::SrvSetFeedbackFrequency, this, _1, _2));
}
  
void HebirosGazeboGroup::UpdateFeedback(const ros::Duration& iteration_time) {
  int i = 0;
  for (auto joint : joints_) {

    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - start_time_;
    ros::Duration feedback_time = current_time - prev_feedback_time_;

    //joint->SetProvideFeedback(true);
    //double velocity = joint->GetVelocity(0);

    feedback_.position[i] = joint->getPositionFbk();
    feedback_.velocity[i] = joint->getVelocityFbk();
    feedback_.effort[i] = joint->getEffortFbk();

    const auto& accel = joint->getAccelerometer();
    feedback_.accelerometer[i].x = accel.x();
    feedback_.accelerometer[i].y = accel.y();
    feedback_.accelerometer[i].z = accel.z();
    const auto& gyro = joint->getGyro();
    feedback_.gyro[i].x = gyro.x();
    feedback_.gyro[i].y = gyro.y();
    feedback_.gyro[i].z = gyro.z();

    // Add temperature feedback
    feedback_.motor_winding_temperature[i] = joint->getTemperature().getMotorWindingTemperature();
    feedback_.motor_housing_temperature[i] = joint->getTemperature().getMotorHousingTemperature();
    feedback_.board_temperature[i] = joint->getTemperature().getActuatorBodyTemperature();

    // Command feedback
    feedback_.position_command[i] = joint->getPositionCmd();
    feedback_.velocity_command[i] = joint->getVelocityCmd();
    feedback_.effort_command[i] = joint->getEffortCmd();

    if (!feedback_pub_.getTopic().empty() &&
      feedback_time.toSec() >= 1.0/feedback_frequency_) {

      feedback_pub_.publish(feedback_);
      prev_feedback_time_ = current_time;
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

void HebirosGazeboGroup::SubCommand(const boost::shared_ptr<hebiros::CommandMsg const> data) {

  if (check_acknowledgement_) {
    acknowledgement_ = true;
    check_acknowledgement_ = false;
  }

  ros::Time current_time = ros::Time::now();
  start_time_ = current_time;
  prev_time_ = current_time;
  ros::Duration elapsed_time = current_time - start_time_;

  for (int i = 0; i < data->name.size(); i++) {
    std::string joint_name = data->name[i];

    auto joint_it = std::find_if(joints_.begin(), joints_.end(), [&joint_name](auto j) { return j->getName() == joint_name; } );

    if (joint_it != joints_.end()) {
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
        joint->setCommand(p_cmd, v_cmd, e_cmd, sender_id, command_lifetime_/1000.0, current_time.toSec());
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

  check_acknowledgement_ = true;

  if (acknowledgement_) {
    check_acknowledgement_ = false;
    acknowledgement_ = false;
    return true;
  }
  else {
    return false;
  }
}

bool HebirosGazeboGroup::SrvSetCommandLifetime(hebiros::SetCommandLifetimeSrv::Request &req,
  hebiros::SetCommandLifetimeSrv::Response &res) {

  command_lifetime_ = req.command_lifetime;
  return true;
}

bool HebirosGazeboGroup::SrvSetFeedbackFrequency(hebiros::SetFeedbackFrequencySrv::Request &req,
  hebiros::SetFeedbackFrequencySrv::Response &res) {

  feedback_frequency_ = req.feedback_frequency;
  return true;
}

}
}
}
