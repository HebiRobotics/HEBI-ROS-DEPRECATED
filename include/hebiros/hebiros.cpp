#include "hebiros.hpp"


//Initialize the hebiros_node and advertise base level topics and services
//Loop in place allowing callback functions to be run
Hebiros_Node::Hebiros_Node (int argc, char **argv) {
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  services["/hebiros/entry_list"] = n.advertiseService(
    "/hebiros/entry_list", &Hebiros_Node::srv_entry_list, this);
      
  services["/hebiros/add_group_from_names"] = n.advertiseService(
    "/hebiros/add_group_from_names", &Hebiros_Node::srv_add_group_from_names, this);
      
  n.param<int>("/hebiros/feedback_frequency", feedback_frequency, 100);
      
  n.param<int>("/hebiros/command_lifetime", command_lifetime, 100);

  ROS_INFO("Parameters:");
  ROS_INFO("/hebiros/feedback_frequency=%d", feedback_frequency);
  ROS_INFO("/hebiros/command_lifetime=%d", command_lifetime);

  loop();
}

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
  groups[req.group_name] = lookup.getGroupFromNames(req.families, req.names);

  if (groups[req.group_name]) {
    register_group(req.group_name);

    ROS_INFO("Created group [%s]:", req.group_name.c_str());
      for (int i = 0; i < req.families.size(); i++) {
        for (int j = 0; j < req.names.size(); j++) {
  	  ROS_INFO("/%s/%s/%s", req.group_name.c_str(),
            req.families[i].c_str(), req.names[j].c_str());
	}
      }

      return true;
    }
  else {
    return false;
  }
}

//Subscriber callback which receives a joint state and sends that as a command to a group
void Hebiros_Node::sub_command(const boost::shared_ptr<sensor_msgs::JointState const> data,
  std::string group_name) {
  std::shared_ptr<Group> group = groups[group_name];
  GroupCommand group_command(group->size());

  Eigen::VectorXd position(group->size());
  Eigen::VectorXd velocity(group->size());
  Eigen::VectorXd effort(group->size());
  for (int i = 0; i < group->size(); i++) {
    if (i < data->position.size()) {
      position(i) = data->position[i];
    }
    if (i < data->velocity.size()) {
      velocity(i) = data->velocity[i];
    }
    if (i < data->effort.size()) {
      effort(i) = data->effort[i];
    }
  }

  group_command.setPosition(position);
  group_command.setVelocity(velocity);
  group_command.setEffort(effort);
  group->sendCommand(group_command);
}

//Service callback which returns the size of a group
bool Hebiros_Node::srv_size(
  SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name) {
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
  std::shared_ptr<Group> group = groups[group_name];

  group->setFeedbackFrequencyHz(req.feedback_frequency);

  ROS_INFO("/hebiros/%s feedback_frequency=%d", group_name.c_str(), req.feedback_frequency);
  return true;
}

//Service callback which allows the command lifetime of a group to be set
bool Hebiros_Node::srv_set_command_lifetime(
  SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
  std::string group_name) {
  std::shared_ptr<Group> group = groups[group_name];

  group->setCommandLifetimeMs(req.command_lifetime);

  ROS_INFO("/hebiros/%s command_lifetime=%d", group_name.c_str(), req.command_lifetime);
  return true;
}

//Advertise topics and services for a group
//Setup a feedback handler to receive feedback from a group
void Hebiros_Node::register_group(std::string group_name) {

  publishers["/hebiros/"+group_name+"/feedback"] =
    n.advertise<FeedbackMsg>("/hebiros/"+group_name+"/feedback", 100);

  publishers["/hebiros/"+group_name+"/feedback/joint_state"] =
    n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/feedback/joint_state", 100);

  subscribers["/hebiros/"+group_name+"/command/joint_state"] = 
    n.subscribe<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100,
    boost::bind(&Hebiros_Node::sub_command, this, _1, group_name));

  services["/hebiros/"+group_name+"/size"] =
    n.advertiseService<SizeSrv::Request, SizeSrv::Response>("/hebiros/"+group_name+"/size",
    boost::bind(&Hebiros_Node::srv_size, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/set_feedback_frequency"] =
    n.advertiseService<SetFeedbackFrequencySrv::Request,
    SetFeedbackFrequencySrv::Response>(
    "/hebiros/"+group_name+"/set_feedback_frequency",
    boost::bind(&Hebiros_Node::srv_set_feedback_frequency, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/set_command_lifetime"] =
    n.advertiseService<SetCommandLifetimeSrv::Request,
    SetCommandLifetimeSrv::Response>(
    "/hebiros/"+group_name+"/set_command_lifetime",
    boost::bind(&Hebiros_Node::srv_set_command_lifetime, this, _1, _2, group_name));

  std::shared_ptr<Group> group = groups[group_name];
  group_infos[group_name] = new GroupInfo(group->size());
  group->requestInfo(group_infos[group_name]);

  group->addFeedbackHandler([this, group_name](const GroupFeedback& group_fbk) {
    this->publish_group(group_name, group_fbk);
  });

  group->setFeedbackFrequencyHz(feedback_frequency);
  group->setCommandLifetimeMs(command_lifetime);
}

//Feedback handler which publishes feedback topics for a group
void Hebiros_Node::publish_group(std::string group_name, const GroupFeedback& group_fbk) {

  FeedbackMsg feedback_msg;
  sensor_msgs::JointState joint_state_msg;

  sensor_msgs::Imu imu_msg;
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[0] = -1;

  for (int i = 0; i < group_fbk.size(); i++) {
    std::string name = (*group_infos[group_name])[i].settings().name().get();
    std::string family = (*group_infos[group_name])[i].settings().family().get();
    float position = group_fbk[i].actuator().position().get();
    float velocity = group_fbk[i].actuator().velocity().get();
    float effort = group_fbk[i].actuator().effort().get();

    joint_state_msg.name.push_back("/"+family+"/"+name);
    joint_state_msg.position.push_back(position);
    joint_state_msg.velocity.push_back(velocity);
    joint_state_msg.effort.push_back(effort);

    hebi::Vector3f accelerometer = group_fbk[i].imu().accelerometer().get();
    hebi::Vector3f gyro = group_fbk[i].imu().gyro().get();
    imu_msg.linear_acceleration.x = accelerometer.getX();
    imu_msg.linear_acceleration.y = accelerometer.getY();
    imu_msg.linear_acceleration.z = accelerometer.getZ();
    imu_msg.angular_velocity.x = gyro.getX();
    imu_msg.angular_velocity.y = gyro.getY();
    imu_msg.angular_velocity.z = gyro.getZ();
    feedback_msg.imu_vector.push_back(imu_msg);
  }

  feedback_msg.joint_state = joint_state_msg;

  publishers["/hebiros/"+group_name+"/feedback"].publish(feedback_msg);
  publishers["/hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
} 

//Deconstruction of a group
void Hebiros_Node::unregister_group(std::string group_name) {
  std::shared_ptr<Group> group = groups[group_name];
  group->clearFeedbackHandlers();
}

void Hebiros_Node::cleanup() {
  for (auto group_pair : groups) {
    unregister_group(group_pair.first);
  }
}


void Hebiros_Node::loop() {
  ros::Rate loop_rate(200);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  cleanup();
}


