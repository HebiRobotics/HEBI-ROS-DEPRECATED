#include "hebiros.hpp"


//Return the entry list which contains every module on the network
//And entry should contain each module's name, family, and mac address
bool Hebiros_Node::srv_entry_list(
  EntryListSrv::Request &req, EntryListSrv::Response &res) {
  EntryListMsg entry_list_msg;
  EntryMsg entry_msg;

  entry_list = lookup.getEntryList();
  entry_list_msg.size = entry_list->size();

  ROS_INFO("Entry list:");
  for (int i = 0; i < entry_list->size(); ++i) {
    auto entry = entry_list->getEntry(i);
    entry_msg.name = entry.name_;
    entry_msg.family = entry.family_;
    entry_msg.mac_address = 0;

    ROS_INFO("/%s/%s", entry_msg.family.c_str(), entry_msg.name.c_str());
    entry_list_msg.entries.push_back(entry_msg);
  }

  res.entry_list = entry_list_msg;
  return true;
}

//Given a group name, list of families, and list of names,
//create and store a shared pointer to a single group
bool Hebiros_Node::srv_add_group_from_names(
  AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res) {

  if (group_joints.find(req.group_name) != group_joints.end()) {
    ROS_WARN("Group [%s] already exists", req.group_name.c_str());
    return true;
  }

  if (!use_gazebo) {
    if (!(groups[req.group_name] = lookup.getGroupFromNames(req.families, req.names))) {
      ROS_WARN("Lookup of group [%s] failed", req.group_name.c_str());
      return false;
    }
  }

  int joint_index = 0;
  ROS_INFO("Created group [%s]:", req.group_name.c_str());
  for (int i = 0; i < req.families.size(); i++) {
    for (int j = 0; j < req.names.size(); j++) {
      std::string joint_name = req.families[i]+"/"+req.names[j];
      
      ROS_INFO("/%s/%s/%s", req.group_name.c_str(),
        req.families[i].c_str(), req.names[j].c_str());
        group_joints[req.group_name][joint_name] = joint_index;

        if (use_gazebo) {

          subscribers["/hebiros_gazebo_plugin/feedback/"+joint_name] =
            n.subscribe<sensor_msgs::JointState>("/hebiros_gazebo_plugin/feedback/"+joint_name,
            100, boost::bind(&Hebiros_Node::sub_publish_group_gazebo,
            this, _1, req.group_name, joint_name));
        }

        joint_index++;
    }
  }

  register_group(req.group_name);
  return true;
}

//Given a group name and a robot_description on the parameter server,
//create and store a shared pointer to a single group
bool Hebiros_Node::srv_add_group_from_urdf(
  AddGroupFromUrdfSrv::Request &req, AddGroupFromUrdfSrv::Response &res) {

  std::string urdf_name("robot_description");
  urdf::Model urdf_model;
  if (!urdf_model.initParam(urdf_name))
  {
    ROS_WARN("Could not load robot_description");
    return false;
  }

  std::set<std::string> joint_names;
  std::set<std::string> family_names;
  std::set<std::string> joint_full_names;
  add_joint_children(joint_names, family_names, joint_full_names, urdf_model.getRoot().get());

  AddGroupFromNamesSrv::Request names_req;
  AddGroupFromNamesSrv::Response names_res;
  names_req.group_name = req.group_name;
  names_req.families.insert(names_req.families.end(), family_names.begin(), family_names.end());
  names_req.names.insert(names_req.names.end(), joint_names.begin(), joint_names.end());
  return Hebiros_Node::srv_add_group_from_names(names_req, names_res);
}



//Service callback which returns the size of a group
bool Hebiros_Node::srv_size(
  SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name) {
  if (use_gazebo) {
    res.size = group_joints[group_name].size();
    return true;
  }

  std::shared_ptr<Group> group = groups[group_name];

  if (group) {
    res.size = group->size();

    ROS_INFO("/hebiros/%s size=%d", group_name.c_str(), res.size);
    return true;
  }
  else {
    return false;
  }
}

//Service callback which allows the feedback frequency of a group to be set
bool Hebiros_Node::srv_set_feedback_frequency(
  SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res,
  std::string group_name) {
  if (use_gazebo) {
    return true;
  }

  std::shared_ptr<Group> group = groups[group_name];

  group->setFeedbackFrequencyHz(req.feedback_frequency);

  ROS_INFO("/hebiros/%s feedback_frequency=%d", group_name.c_str(), req.feedback_frequency);
  return true;
}

//Service callback which allows the command lifetime of a group to be set
bool Hebiros_Node::srv_set_command_lifetime(
  SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
  std::string group_name) {
  if (use_gazebo) {
    SetCommandLifetimeSrv srv;
    srv.request.command_lifetime = req.command_lifetime;

    clients["/hebiros_gazebo_plugin/set_command_lifetime"].call(srv);
  }
  else {
    std::shared_ptr<Group> group = groups[group_name];

    group->setCommandLifetimeMs(req.command_lifetime);
  }

  ROS_INFO("/hebiros/%s command_lifetime=%d", group_name.c_str(), req.command_lifetime);
  return true;
}

//Service callback which sends a command and indicates whether it has been received
bool Hebiros_Node::srv_send_command_with_acknowledgement(
  SendCommandWithAcknowledgementSrv::Request &req, SendCommandWithAcknowledgementSrv::Response &res,
  std::string group_name) {

  if (use_gazebo) {

    if (names_in_order(req.command)) {
      
      std_srvs::Empty empty_srv;

      ros::ServiceClient acknowledge_client = n.serviceClient<std_srvs::Empty>(
      "/hebiros_gazebo_plugin/acknowledge");

      while(!acknowledge_client.call(empty_srv)) {
        publishers["/hebiros_gazebo_plugin/command"].publish(req.command);
      }
      return true;
    }
    else {
      ROS_WARN("Simulated commands are assigned with different orders.  Command will not be sent.");
      return false;
    }
  }

  std::shared_ptr<Group> group = groups[group_name];
  GroupCommand group_command(group->size());

  sensor_msgs::JointState joint_data;
  joint_data.name = req.command.name;
  joint_data.position = req.command.position;
  joint_data.velocity = req.command.velocity;
  joint_data.effort = req.command.effort;
  SettingsMsg settings_data;
  settings_data = req.command.settings;

  add_joint_command(&group_command, joint_data, group_name);
  add_settings_command(&group_command, settings_data, group_name);

  return group->sendCommandWithAcknowledgement(group_command);
}


