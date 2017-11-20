#include "hebiros.hpp"


//Initialize the hebiros_node and advertise base level topics and services
//Loop in place allowing callback functions to be run
Hebiros_Node::Hebiros_Node (int argc, char **argv) {

  use_gazebo = false;

  for (int i = 2; i < argc; i += 2) {
    if (argv[i-1] == std::string("-use_gazebo")) {
      if (argv[i] == std::string("true")) {
        use_gazebo = true;
        ROS_INFO("Using Gazebo");
      }
    }
  }

  if (!use_gazebo) {
    n.setParam("/use_sim_time", false);
  }

  services["/hebiros/entry_list"] = n.advertiseService(
    "/hebiros/entry_list", &Hebiros_Node::srv_entry_list, this);
      
  services["/hebiros/add_group_from_names"] = n.advertiseService(
    "/hebiros/add_group_from_names", &Hebiros_Node::srv_add_group_from_names, this);

  services["/hebiros/add_group_from_urdf"] = n.advertiseService(
    "/hebiros/add_group_from_urdf", &Hebiros_Node::srv_add_group_from_urdf, this);

  n.param<int>("/hebiros/node_frequency", node_frequency, 200);
  n.setParam("/hebiros/node_frequency", node_frequency);

  n.param<int>("/hebiros/action_frequency", action_frequency, 200);
  n.setParam("/hebiros/action_frequency", action_frequency);

  n.param<int>("/hebiros/feedback_frequency", feedback_frequency, 100);
  n.setParam("/hebiros/feedback_frequency", feedback_frequency);
      
  n.param<int>("/hebiros/command_lifetime", command_lifetime, 100);
  n.setParam("/hebiros/command_lifetime", command_lifetime);

  ROS_INFO("Parameters:");
  ROS_INFO("/hebiros/node_frequency=%d", node_frequency);
  ROS_INFO("/hebiros/action_frequency=%d", action_frequency);
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

  if (groups[req.group_name]) {
    ROS_WARN("Group [%s] already exists", req.group_name.c_str());
    return true;
  }

  if (!use_gazebo) {
    if (!(groups[req.group_name] = lookup.getGroupFromNames(req.families, req.names))) {
      ROS_WARN("Lookup of group [%s] failed", req.group_name.c_str());
      return false;
    }
  }
  else {
    std::string type;
    int publish_rate;

    n.param<std::string>("/hebiros/"+req.group_name+"/joint_state_controller/type", type,
      "joint_state_controller/JointStateController");
    n.setParam("/hebiros/"+req.group_name+"/joint_state_controller/type", type);

    n.param<int>("/hebiros/"+req.group_name+"/joint_state_controller/publish_rate", 
      publish_rate, 50);
    n.setParam("/hebiros/"+req.group_name+"/joint_state_controller/publish_rate", publish_rate);
  }

  int joint_index = 0;
  ROS_INFO("Created group [%s]:", req.group_name.c_str());
  for (int i = 0; i < req.families.size(); i++) {
    for (int j = 0; j < req.names.size(); j++) {
      ROS_INFO("/%s/%s/%s", req.group_name.c_str(),
        req.families[i].c_str(), req.names[j].c_str());
        group_joints[req.group_name][req.families[i]+"/"+req.names[j]] = joint_index;

        if (use_gazebo) {
          std::string controller_namespace = "/hebiros/"+req.group_name+"/"+
            req.families[i]+"/"+req.names[j]+"/controller";

          std::string type;
          float p, i, d;
          int publish_rate;

          n.setParam(controller_namespace+"/joint", req.families[i]+"/"+req.names[j]);

          n.param<std::string>(controller_namespace+"/type", type,
            "effort_controllers/JointEffortController");
          n.setParam(controller_namespace+"/type", type);

          n.param<float>(controller_namespace+"/pid/p", p, 100.0);
          n.setParam(controller_namespace+"/pid/p", p);

          n.param<float>(controller_namespace+"/pid/i", i, 0.01);
          n.setParam(controller_namespace+"/pid/i", i);

          n.param<float>(controller_namespace+"/pid/d", d, 10.0);
          n.setParam(controller_namespace+"/pid/d", d);

          n.param<int>(controller_namespace+"/publish_rate", publish_rate, 50);
          n.setParam(controller_namespace+"/publish_rate", publish_rate);
        }

        joint_index++;
    }
  }

  register_group(req.group_name);
  return true;
}


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

//Given a group name and a robot_description on the parameter server,
//create and store a shared pointer to a single group
bool Hebiros_Node::srv_add_group_from_urdf(
  AddGroupFromUrdfSrv::Request &req, AddGroupFromUrdfSrv::Response &res) {

  std::string urdf_name("robot_description");
  urdf::Model urdf_model;
  if (!urdf_model.initParam(urdf_name))
  {
    ROS_INFO("Could not load robot_description");
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

//Subscriber callback which receives a joint state and sends that as a command to a group
void Hebiros_Node::sub_command(const boost::shared_ptr<sensor_msgs::JointState const> data,
  std::string group_name) {

  if (use_gazebo) {
    std_msgs::Float64 command_msg;
    for (int i = 0; i < data->name.size(); i++) {
      std::string joint_name = data->name[i];
      command_msg.data = data->effort[i];
      publishers["/hebiros/"+group_name+"/"+joint_name+"/controller/command"].publish(
        command_msg);
    }
    return;
  }

  std::shared_ptr<Group> group = groups[group_name];
  GroupCommand group_command(group->size());

  Eigen::VectorXd position(group->size());
  Eigen::VectorXd velocity(group->size());
  Eigen::VectorXd effort(group->size());
  for (int i = 0; i < group->size(); i++) {
    if (i < data->position.size()) {
      position(i) = data->position[i];
    } else {
      position(i) = std::numeric_limits<double>::quiet_NaN();
    }
    if (i < data->velocity.size()) {
      velocity(i) = data->velocity[i];
    } else {
      velocity(i) = std::numeric_limits<double>::quiet_NaN();
    }
    if (i < data->effort.size()) {
      effort(i) = data->effort[i];
    } else {
      effort(i) = std::numeric_limits<double>::quiet_NaN();
    }
  }

  group_command.setPosition(position);
  group_command.setVelocity(velocity);
  group_command.setEffort(effort);
  group->sendCommand(group_command);
}


//Subscriber callback which publishes feedback topics for a group in gazebo
void Hebiros_Node::sub_publish_group_gazebo(const boost::shared_ptr<sensor_msgs::JointState const>
  data, std::string group_name) {
  int size = data->name.size();

  FeedbackMsg feedback_msg;
  feedback_msg.imu_vector.resize(size);

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name.resize(size);
  joint_state_msg.position.resize(size);
  joint_state_msg.velocity.resize(size);
  joint_state_msg.effort.resize(size);

  sensor_msgs::Imu imu_msg;
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[0] = -1;

  for (int i = 0; i < size; i++) {

    std::string joint_name = data->name[i];
    double position = data->position[i];
    double velocity = data->velocity[i];
    double effort = data->effort[i];
    int joint_index = group_joints[group_name][joint_name];

    joint_state_msg.name[joint_index] = joint_name;
    joint_state_msg.position[joint_index] = position;
    joint_state_msg.velocity[joint_index] = velocity;
    joint_state_msg.effort[joint_index] = effort;

    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 0;
    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;
    feedback_msg.imu_vector[joint_index] = imu_msg;
  }

  feedback_msg.joint_state = joint_state_msg;

  publishers["/hebiros/"+group_name+"/feedback"].publish(feedback_msg);
  publishers["/hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
  group_feedback_msgs[group_name] = feedback_msg;
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
    return true;
  }

  std::shared_ptr<Group> group = groups[group_name];

  group->setCommandLifetimeMs(req.command_lifetime);

  ROS_INFO("/hebiros/%s command_lifetime=%d", group_name.c_str(), req.command_lifetime);
  return true;
}

//Action callback which controls following a trajectory
void Hebiros_Node::action_trajectory(const TrajectoryGoalConstPtr& goal, std::string group_name) {

  std::shared_ptr<actionlib::SimpleActionServer<TrajectoryAction>> action_server =
    trajectory_actions[group_name];

  int num_waypoints = goal->waypoints.size();
  if (num_waypoints < 1) {
    return;
  }
  int num_joints = goal->waypoints[0].names.size();

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  Eigen::VectorXd time(num_waypoints);

  for (int i = 0; i < num_waypoints; i++) {
    time(i) = goal->times[i];
  }

  for (int i = 0; i < num_joints; i++) {
    std::string joint_name = goal->waypoints[0].names[i];
    int joint_index = group_joints[group_name][joint_name];

    for (int j = 0; j < num_waypoints; j++) {
      double position = goal->waypoints[j].positions[i];
      double velocity = goal->waypoints[j].velocities[i];
      double acceleration = goal->waypoints[j].accelerations[i];

      positions(joint_index, j) = position;
      velocities(joint_index, j) = velocity;
      accelerations(joint_index, j) = acceleration;
    }
  }

  auto trajectory = trajectory::Trajectory::createUnconstrainedQp(
    time, positions, &velocities, &accelerations);
  Eigen::VectorXd position_command(num_joints);
  Eigen::VectorXd velocity_command(num_joints);

  double trajectory_duration = trajectory->getDuration();
  double previous_time;
  double current_time;
  double loop_duration;
  TrajectoryFeedback feedback;

  ros::Rate loop_rate(action_frequency);

  ROS_INFO("Group [%s]: executing trajectory", group_name.c_str());
  previous_time = ros::Time::now().toSec();
  for (double t = 0; t < trajectory_duration; t += loop_duration)
  {
    if (action_server->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("Group [%s]: Preempted trajectory", group_name.c_str());
      action_server->setPreempted();
      return;
    }

    feedback.percent_complete = (t / trajectory_duration) * 100;
    action_server->publishFeedback(feedback);

    trajectory->getState(t, &position_command, &velocity_command, nullptr);
    sensor_msgs::JointState command_msg;
    command_msg.name.resize(num_joints);
    command_msg.position.resize(num_joints);
    command_msg.velocity.resize(num_joints);

    for (int i = 0; i < num_joints; i++) {
      std::string joint_name = goal->waypoints[0].names[i];
      int joint_index = group_joints[group_name][joint_name];
      command_msg.name[joint_index] = joint_name;
      command_msg.position[joint_index] = position_command(i);
      command_msg.velocity[joint_index] = velocity_command(i);
    }
    publishers["/hebiros/"+group_name+"/command/joint_state"].publish(command_msg);

    ros::spinOnce();
    loop_rate.sleep();
    current_time = ros::Time::now().toSec();
    loop_duration = current_time - previous_time;
    previous_time = current_time;
  }

  TrajectoryResult result;
  result.final_state = group_feedback_msgs[group_name].joint_state;
  action_server->setSucceeded(result);
  ROS_INFO("Group [%s]: Finished executing trajectory", group_name.c_str());
}

//Advertise topics and services for a group
//Setup a feedback handler to receive feedback from a group
void Hebiros_Node::register_group(std::string group_name) {

  publishers["/hebiros/"+group_name+"/feedback"] =
    n.advertise<FeedbackMsg>("/hebiros/"+group_name+"/feedback", 100);

  publishers["/hebiros/"+group_name+"/feedback/joint_state"] =
    n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/feedback/joint_state", 100);

  publishers["/hebiros/"+group_name+"/command/joint_state"] =   
    n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100);

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

  trajectory_actions[group_name] = std::make_shared<
    actionlib::SimpleActionServer<TrajectoryAction>>(
    n, "/hebiros/"+group_name+"/trajectory",
    boost::bind(&Hebiros_Node::action_trajectory, this, _1, group_name), false);

  trajectory_actions[group_name]->start();

  if (!use_gazebo) {
    std::shared_ptr<Group> group = groups[group_name];
    group_infos[group_name] = new GroupInfo(group->size());
    group->requestInfo(group_infos[group_name]);

    group->addFeedbackHandler([this, group_name](const GroupFeedback& group_fbk) {
      this->publish_group(group_name, group_fbk);
    });

    group->setFeedbackFrequencyHz(feedback_frequency);
    group->setCommandLifetimeMs(command_lifetime);
  }
  else {
    subscribers["/hebiros/"+group_name+"/joint_states"] =
      n.subscribe<sensor_msgs::JointState>("/hebiros/"+group_name+"/joint_states", 100,
      boost::bind(&Hebiros_Node::sub_publish_group_gazebo, this, _1, group_name));

    for (auto group_joints_pair : group_joints[group_name]) {
      publishers["/hebiros/"+group_name+"/"+group_joints_pair.first+"/controller/command"] =
        n.advertise<std_msgs::Float64>(
        "/hebiros/"+group_name+"/"+group_joints_pair.first+"/controller/command", 100);
    }
  }
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

    joint_state_msg.name.push_back(family+"/"+name);
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
  group_feedback_msgs[group_name] = feedback_msg;
} 

//Deconstruction of a group
void Hebiros_Node::unregister_group(std::string group_name) {
  if (groups[group_name]) {
    std::shared_ptr<Group> group = groups[group_name];
    group->clearFeedbackHandlers();
  }
}

void Hebiros_Node::cleanup() {
  for (auto group_pair : groups) {
    unregister_group(group_pair.first);
  }
}


void Hebiros_Node::loop() {
  ros::Rate loop_rate(node_frequency);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  cleanup();
}


