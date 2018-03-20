#include "hebiros.hpp"


//Split a joint into name and family by '/'
bool Hebiros_Node::split(const std::string &orig, std::string &name, std::string &family)
{
  std::stringstream ss(orig);

  if (!std::getline(ss, family, '/')) {
    return false;
  }

  std::getline(ss, name);
  return true;
}

//Creates a list of names and families from joints in a urdf
void Hebiros_Node::add_joint_children(std::set<std::string>& names, std::set<std::string>& families, std::set<std::string>& full_names, const urdf::Link* link)
{
  for (auto& joint : link->child_joints) {

    if (joint->type != urdf::Joint::FIXED) {
      std::string name, family;

      if (split(joint->name, name, family)) {
        names.insert(name);
        families.insert(family);
        full_names.insert(joint->name);
      }
    }
  }

  for (auto& link_child : link->child_links) {
    add_joint_children(names, families, full_names, link_child.get());
  }
}

//Determine whether a CommandMsg has valid lists of names
bool Hebiros_Node::names_in_order(CommandMsg command_msg) {
  return (command_msg.name == command_msg.settings.name ||
    command_msg.settings.name.size() == 0) &&
    (command_msg.name == command_msg.settings.position_gains.name || 
    command_msg.settings.position_gains.name.size() == 0) &&
    (command_msg.name == command_msg.settings.velocity_gains.name ||
    command_msg.settings.velocity_gains.name.size() == 0) &&
    (command_msg.name == command_msg.settings.effort_gains.name ||
    command_msg.settings.effort_gains.name.size() == 0);
}

//Determine whether a joint is in a specific group
bool Hebiros_Node::joint_found(std::string group_name, std::string joint_name) {
  return group_joints[group_name].find(joint_name) != group_joints[group_name].end();
}

//Print a warning indicating that a joint could not be found
void Hebiros_Node::joint_not_found(std::string joint_name) {
  ROS_WARN("Unable to find joint: %s.  Command will not be sent.", joint_name.c_str());
}

//Add joint state values to a group command
void Hebiros_Node::add_joint_command(GroupCommand* group_command,
  sensor_msgs::JointState data, std::string group_name) {

  for (int i = 0; i < data.name.size(); i++) {
    if (joint_found(group_name, data.name[i])) {

      int joint_index = group_joints[group_name][data.name[i]];

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
      joint_not_found(data.name[i]);
    }
  }
}

//Add settings values to a group command
void Hebiros_Node::add_settings_command(GroupCommand* group_command,
  SettingsMsg data, std::string group_name) {

  for (int i = 0; i < data.name.size(); i++) {
    if (joint_found(group_name, data.name[i])) {

      int joint_index = group_joints[group_name][data.name[i]];

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
      joint_not_found(data.name[i]);
    }
  }

  add_position_gains_command(group_command, data.position_gains, group_name);
  add_velocity_gains_command(group_command, data.velocity_gains, group_name);
  add_effort_gains_command(group_command, data.effort_gains, group_name);
}

//Add position gain values to a group command
void Hebiros_Node::add_position_gains_command(GroupCommand* group_command,
  PidGainsMsg data, std::string group_name) {

  for (int i = 0; i < data.name.size(); i++) {
    if (joint_found(group_name, data.name[i])) {

      int joint_index = group_joints[group_name][data.name[i]];

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
      joint_not_found(data.name[i]);
    }
  }
}

//Add velocity gain values to a group command
void Hebiros_Node::add_velocity_gains_command(GroupCommand* group_command,
  PidGainsMsg data, std::string group_name) {

  for (int i = 0; i < data.name.size(); i++) {
    if (joint_found(group_name, data.name[i])) {

      int joint_index = group_joints[group_name][data.name[i]];

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
      joint_not_found(data.name[i]);
    }
  }
}

//Add effort gain values to a group command
void Hebiros_Node::add_effort_gains_command(GroupCommand* group_command,
  PidGainsMsg data, std::string group_name) {

  std::shared_ptr<Group> group = groups[group_name];

  for (int i = 0; i < data.name.size(); i++) {
    if (joint_found(group_name, data.name[i])) {

      int joint_index = group_joints[group_name][data.name[i]];

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
      joint_not_found(data.name[i]);
    }
  }
}


