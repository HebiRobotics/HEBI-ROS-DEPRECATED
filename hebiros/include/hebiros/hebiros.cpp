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


