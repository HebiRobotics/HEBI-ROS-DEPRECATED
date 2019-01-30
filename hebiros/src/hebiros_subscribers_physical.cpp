#include "hebiros_subscribers_physical.h"
#include "hebiros_group_registry.h"
#include "hebiros.h"

using namespace hebi;


void HebirosSubscribersPhysical::registerGroupSubscribers(std::string group_name) {

  subscribers["hebiros/"+group_name+"/command"] =
    HebirosNode::n_ptr->subscribe<CommandMsg>("hebiros/"+group_name+"/command", 100,
    boost::bind(&HebirosSubscribersPhysical::command, this, _1, group_name));

  subscribers["hebiros/"+group_name+"/command/joint_state"] =
    HebirosNode::n_ptr->subscribe<sensor_msgs::JointState>(
    "hebiros/"+group_name+"/command/joint_state", 100,
    boost::bind(&HebirosSubscribersPhysical::jointCommand, this, _1, group_name));

  // TODO: replace with better abstraction later
  HebirosGroupPhysical* group = dynamic_cast<HebirosGroupPhysical*>
    (hebiros::HebirosGroupRegistry::Instance().getGroup(group_name));
  if (!group) {
    ROS_WARN("Improper group type during register subscribers call");
    return;
  }
  
  group->group_ptr->requestInfo(group->group_info);

  group->group_ptr->addFeedbackHandler([this, group_name](const GroupFeedback& group_fbk) {
    this->feedback(group_name, group_fbk);
  });

  group->setFeedbackFrequency(
    HebirosParameters::getInt("hebiros/feedback_frequency"));
  group->setCommandLifetime(
    HebirosParameters::getInt("hebiros/command_lifetime"));
}

void HebirosSubscribersPhysical::command(const boost::shared_ptr<CommandMsg const> data,
  std::string group_name) {

  // TODO: replace with better abstraction later -- move send command into group
  HebirosGroupPhysical* group = dynamic_cast<HebirosGroupPhysical*>
    (hebiros::HebirosGroupRegistry::Instance().getGroup(group_name)); 
  if (!group) {
    ROS_WARN("Improper group type during command call");
    return;
  }

  sensor_msgs::JointState joint_data;
  joint_data.name = data->name;
  joint_data.position = data->position;
  joint_data.velocity = data->velocity;
  joint_data.effort = data->effort;
  SettingsMsg settings_data;
  settings_data = data->settings;

  GroupCommand group_command(group->size);
  addJointCommand(&group_command, joint_data, group_name);
  addSettingsCommand(&group_command, settings_data, group_name);

  group->group_ptr->sendCommand(group_command);
}

void HebirosSubscribersPhysical::jointCommand(
  const boost::shared_ptr<sensor_msgs::JointState const> data, std::string group_name) {

  // TODO: replace with better abstraction later -- move send command into group
  HebirosGroupPhysical* group = dynamic_cast<HebirosGroupPhysical*>
    (hebiros::HebirosGroupRegistry::Instance().getGroup(group_name)); 
  if (!group) {
    ROS_WARN("Improper group type during jointCommand call");
    return;
  }

  sensor_msgs::JointState joint_data;
  joint_data.name = data->name;
  joint_data.position = data->position;
  joint_data.velocity = data->velocity;
  joint_data.effort = data->effort;

  GroupCommand group_command(group->size);
  addJointCommand(&group_command, joint_data, group_name);

  group->group_ptr->sendCommand(group_command);
}

void HebirosSubscribersPhysical::addJointCommand(GroupCommand* group_command,
  sensor_msgs::JointState data, std::string group_name) {

  HebirosGroup* group = hebiros::HebirosGroupRegistry::Instance().getGroup(group_name);

  for (int i = 0; i < data.name.size(); i++) {
    if (HebirosSubscribers::jointFound(group_name, data.name[i])) {

      int joint_index = group->joints[data.name[i]];

      if (i < data.position.size()) {
        (*group_command)[joint_index].actuator().position().set(data.position[i]);
      }
      if (i < data.velocity.size()) {
        (*group_command)[joint_index].actuator().velocity().set(data.velocity[i]);
      }
      if (i < data.effort.size()) {
        (*group_command)[joint_index].actuator().effort().set(data.effort[i]);
      }
    }
    else {
      HebirosSubscribers::jointNotFound(data.name[i]);
    }
  }
}

void HebirosSubscribersPhysical::addSettingsCommand(GroupCommand* group_command,
  SettingsMsg data, std::string group_name) {

  HebirosGroup* group = hebiros::HebirosGroupRegistry::Instance().getGroup(group_name);

  for (int i = 0; i < data.name.size(); i++) {
    if (HebirosSubscribers::jointFound(group_name, data.name[i])) {

      int joint_index = group->joints[data.name[i]];

      if (i < data.save_current_settings.size()) {
        if (data.save_current_settings[i]) {
          (*group_command)[joint_index].settings().saveCurrentSettings().set();
        }
      }
      if (i < data.control_strategy.size()) {
        Command::ControlStrategy strategy = static_cast<Command::ControlStrategy>( 
          data.control_strategy[i]);
        (*group_command)[joint_index].
          settings().actuator().controlStrategy().set(strategy);
      }
    }
    else {
      HebirosSubscribers::jointNotFound(data.name[i]);
    }
  }

  addPositionGainsCommand(group_command, data.position_gains, group_name);
  addVelocityGainsCommand(group_command, data.velocity_gains, group_name);
  addEffortGainsCommand(group_command, data.effort_gains, group_name);
}

void HebirosSubscribersPhysical::addPositionGainsCommand(GroupCommand* group_command,
  PidGainsMsg data, std::string group_name) {

  HebirosGroup* group = hebiros::HebirosGroupRegistry::Instance().getGroup(group_name);

  for (int i = 0; i < data.name.size(); i++) {
    if (HebirosSubscribers::jointFound(group_name, data.name[i])) {

      int joint_index = group->joints[data.name[i]];

      if (i < data.kp.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().kP().set(data.kp[i]);
      }
      if (i < data.ki.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().kI().set(data.ki[i]);
      }
      if (i < data.kd.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().kD().set(data.kd[i]);
      }
      if (i < data.feed_forward.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().feedForward().set(data.feed_forward[i]);
      }
      if (i < data.dead_zone.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().deadZone().set(data.dead_zone[i]);
      }
      if (i < data.i_clamp.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().iClamp().set(data.i_clamp[i]);
      }
      if (i < data.punch.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().punch().set(data.punch[i]);
      }
      if (i < data.min_target.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().minTarget().set(data.min_target[i]);
      }
      if (i < data.max_target.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().maxTarget().set(data.max_target[i]);
      }
      if (i < data.target_lowpass.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().targetLowpass().set(data.target_lowpass[i]);
      }
      if (i < data.min_output.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().minOutput().set(data.min_output[i]);
      }
      if (i < data.max_output.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().maxOutput().set(data.max_output[i]);
      }
      if (i < data.output_lowpass.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().outputLowpass().set(data.output_lowpass[i]);
      }
      if (i < data.d_on_error.size()) {
        (*group_command)[joint_index].
          settings().actuator().positionGains().dOnError().set(data.d_on_error[i]);
      }
    }
    else {
      HebirosSubscribers::jointNotFound(data.name[i]);
    }
  }
}

void HebirosSubscribersPhysical::addVelocityGainsCommand(GroupCommand* group_command,
  PidGainsMsg data, std::string group_name) {

  HebirosGroup* group = hebiros::HebirosGroupRegistry::Instance().getGroup(group_name);

  for (int i = 0; i < data.name.size(); i++) {
    if (HebirosSubscribers::jointFound(group_name, data.name[i])) {

      int joint_index = group->joints[data.name[i]];

      if (i < data.kp.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().kP().set(data.kp[i]);
      }
      if (i < data.ki.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().kI().set(data.ki[i]);
      }
      if (i < data.kd.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().kD().set(data.kd[i]);
      }
      if (i < data.feed_forward.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().feedForward().set(data.feed_forward[i]);
      }
      if (i < data.dead_zone.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().deadZone().set(data.dead_zone[i]);
      }
      if (i < data.i_clamp.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().iClamp().set(data.i_clamp[i]);
      }
      if (i < data.punch.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().punch().set(data.punch[i]);
      }
      if (i < data.min_target.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().minTarget().set(data.min_target[i]);
      }
      if (i < data.max_target.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().maxTarget().set(data.max_target[i]);
      }
      if (i < data.target_lowpass.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().targetLowpass().set(data.target_lowpass[i]);
      }
      if (i < data.min_output.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().minOutput().set(data.min_output[i]);
      }
      if (i < data.max_output.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().maxOutput().set(data.max_output[i]);
      }
      if (i < data.output_lowpass.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().outputLowpass().set(data.output_lowpass[i]);
      }
      if (i < data.d_on_error.size()) {
        (*group_command)[joint_index].
          settings().actuator().velocityGains().dOnError().set(data.d_on_error[i]);
      }
    }
    else {
      HebirosSubscribers::jointNotFound(data.name[i]);
    }
  }
}

void HebirosSubscribersPhysical::addEffortGainsCommand(GroupCommand* group_command,
  PidGainsMsg data, std::string group_name) {

  HebirosGroup* group = hebiros::HebirosGroupRegistry::Instance().getGroup(group_name);

  for (int i = 0; i < data.name.size(); i++) {
    if (HebirosSubscribers::jointFound(group_name, data.name[i])) {

      int joint_index = group->joints[data.name[i]];

      if (i < data.kp.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().kP().set(data.kp[i]);
      }
      if (i < data.ki.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().kI().set(data.ki[i]);
      }
      if (i < data.kd.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().kD().set(data.kd[i]);
      }
      if (i < data.feed_forward.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().feedForward().set(data.feed_forward[i]);
      }
      if (i < data.dead_zone.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().deadZone().set(data.dead_zone[i]);
      }
      if (i < data.i_clamp.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().iClamp().set(data.i_clamp[i]);
      }
      if (i < data.punch.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().punch().set(data.punch[i]);
      }
      if (i < data.min_target.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().minTarget().set(data.min_target[i]);
      }
      if (i < data.max_target.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().maxTarget().set(data.max_target[i]);
      }
      if (i < data.target_lowpass.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().targetLowpass().set(data.target_lowpass[i]);
      }
      if (i < data.min_output.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().minOutput().set(data.min_output[i]);
      }
      if (i < data.max_output.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().maxOutput().set(data.max_output[i]);
      }
      if (i < data.output_lowpass.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().outputLowpass().set(data.output_lowpass[i]);
      }
      if (i < data.d_on_error.size()) {
        (*group_command)[joint_index].
          settings().actuator().effortGains().dOnError().set(data.d_on_error[i]);
      }
    }
    else {
      HebirosSubscribers::jointNotFound(data.name[i]);
    }
  }
}


void HebirosSubscribersPhysical::feedback(std::string group_name, const GroupFeedback& group_fbk) {

  // TODO: replace with better abstraction later
  HebirosGroupPhysical* group = dynamic_cast<HebirosGroupPhysical*>
    (hebiros::HebirosGroupRegistry::Instance().getGroup(group_name));
  if (!group) {
    ROS_WARN("Improper group type during feedback call");
    return;
  }

  FeedbackMsg feedback_msg;
  sensor_msgs::JointState joint_state_msg;

  for (int i = 0; i < group_fbk.size(); i++) {
    std::string name = group->group_info[i].settings().name().get();
    std::string family = group->group_info[i].settings().family().get();
    double position = group_fbk[i].actuator().position().get();
    double velocity = group_fbk[i].actuator().velocity().get();
    double effort = group_fbk[i].actuator().effort().get();

    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.push_back(family+"/"+name);
    joint_state_msg.position.push_back(position);
    joint_state_msg.velocity.push_back(velocity);
    joint_state_msg.effort.push_back(effort);

    feedback_msg.name.push_back(family+"/"+name);
    feedback_msg.position.push_back(position);
    feedback_msg.velocity.push_back(velocity);
    feedback_msg.effort.push_back(effort);
    feedback_msg.position_command.push_back(group_fbk[i].actuator().positionCommand().get());
    feedback_msg.velocity_command.push_back(group_fbk[i].actuator().velocityCommand().get());
    feedback_msg.effort_command.push_back(group_fbk[i].actuator().effortCommand().get());
    geometry_msgs::Vector3 accelerometer_msg;
    accelerometer_msg.x = group_fbk[i].imu().accelerometer().get().getX();
    accelerometer_msg.y = group_fbk[i].imu().accelerometer().get().getY();
    accelerometer_msg.z = group_fbk[i].imu().accelerometer().get().getZ();
    feedback_msg.accelerometer.push_back(accelerometer_msg);
    geometry_msgs::Vector3 gyro_msg;
    gyro_msg.x = group_fbk[i].imu().gyro().get().getX();
    gyro_msg.y = group_fbk[i].imu().gyro().get().getY();
    gyro_msg.z = group_fbk[i].imu().gyro().get().getZ();
    feedback_msg.gyro.push_back(gyro_msg);
    geometry_msgs::Quaternion orient_msg;
    const auto& orient = group_fbk[i].imu().orientation().get();
    orient_msg.w = orient.getW();
    orient_msg.x = orient.getX();
    orient_msg.y = orient.getY();
    orient_msg.z = orient.getZ();
    feedback_msg.orientation.push_back(orient_msg);
    feedback_msg.deflection.push_back(group_fbk[i].actuator().deflection().get());
    feedback_msg.deflection_velocity.push_back(group_fbk[i].actuator().deflectionVelocity().get());
    feedback_msg.motor_velocity.push_back(group_fbk[i].actuator().motorVelocity().get());
    feedback_msg.motor_current.push_back(group_fbk[i].actuator().motorCurrent().get());
    feedback_msg.motor_winding_current.push_back(group_fbk[i].actuator().motorWindingCurrent().get());
    feedback_msg.motor_sensor_temperature.push_back(
      group_fbk[i].actuator().motorSensorTemperature().get());
    feedback_msg.motor_winding_temperature.push_back(
      group_fbk[i].actuator().motorWindingTemperature().get());
    feedback_msg.motor_housing_temperature.push_back(
      group_fbk[i].actuator().motorHousingTemperature().get());
    feedback_msg.board_temperature.push_back(group_fbk[i].boardTemperature().get());
    feedback_msg.processor_temperature.push_back(group_fbk[i].processorTemperature().get());
    feedback_msg.voltage.push_back(group_fbk[i].voltage().get());
    std_msgs::ColorRGBA led_msg;
    led_msg.r = group_fbk[i].led().getColor().getRed();
    led_msg.g = group_fbk[i].led().getColor().getGreen();
    led_msg.b = group_fbk[i].led().getColor().getBlue();
    if (group_fbk[i].led().hasColor()) {
      led_msg.a = 255;
    }
    else {
      led_msg.a = 0;
    }
    feedback_msg.led_color.push_back(led_msg);
    feedback_msg.sequence_number.push_back(group_fbk[i].actuator().sequenceNumber().get());
    feedback_msg.receive_time.push_back(group_fbk[i].actuator().receiveTime().get());
    feedback_msg.transmit_time.push_back(group_fbk[i].actuator().transmitTime().get());
    feedback_msg.hardware_receive_time.push_back(group_fbk[i].actuator().hardwareReceiveTime().get());
    feedback_msg.hardware_transmit_time.push_back(
      group_fbk[i].actuator().hardwareTransmitTime().get());
  }

  group->feedback_msg = feedback_msg;
  group->joint_state_msg = joint_state_msg;

  HebirosNode::publishers_physical.feedback(feedback_msg, group_name);
  HebirosNode::publishers_physical.feedbackJointState(joint_state_msg, group_name);
}




