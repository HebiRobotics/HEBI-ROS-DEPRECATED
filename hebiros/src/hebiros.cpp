#include "hebiros.h"


std::shared_ptr<ros::NodeHandle> HebirosNode::n_ptr;
HebirosPublishersGazebo HebirosNode::publishers_gazebo;
HebirosPublishersPhysical HebirosNode::publishers_physical;
HebirosSubscribersGazebo HebirosNode::subscribers_gazebo;
HebirosSubscribersPhysical HebirosNode::subscribers_physical;
HebirosServicesGazebo HebirosNode::services_gazebo;
HebirosServicesPhysical HebirosNode::services_physical;
HebirosClients HebirosNode::clients;
HebirosActions HebirosNode::actions;

//Initialize the hebiros_node and advertise base level topics and services
//Loop in place allowing callback functions to be run
HebirosNode::HebirosNode (int argc, char **argv) {

  HebirosNode::n_ptr = std::make_shared<ros::NodeHandle>(n);

  use_gazebo = false;

  for (int i = 2; i < argc; i += 2) {
    if (argv[i-1] == std::string("-use_gazebo")) {
      if (argv[i] == std::string("true")) {
        use_gazebo = true;
        ROS_INFO("Using Gazebo");
      }
    }
  }

  if (use_gazebo) {
    HebirosParameters::setBool("use_sim_time", true);
    services_gazebo.registerNodeServices();
    clients.registerNodeClients();
  }
  else {
    HebirosParameters::setBool("use_sim_time", false);
    services_physical.registerNodeServices();
  }

  HebirosParameters::setNodeParameters();

  loop();
}

void HebirosNode::cleanup() {
}

void HebirosNode::loop() {
  ros::Rate loop_rate(HebirosParameters::getInt("hebiros/node_frequency"));

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  cleanup();
}


