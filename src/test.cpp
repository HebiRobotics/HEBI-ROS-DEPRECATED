#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
//#include <urdf/model.h>

#include "hebi/HebiEntry.h"
#include "hebi/HebiEntryList.h"
//#include "hebi/HebiFeedback.h"
#include "hebi/entry_list.h"
#include "hebi/add_group_from_names.h"
#include "hebi/size.h"

#include "color.hpp"
#include "command.hpp"
#include "feedback.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "group_info.hpp"
#include "info.hpp"
#include "kinematics.hpp"
#include "log_file.hpp"
#include "lookup.hpp"
#include "mac_address.hpp"
#include "trajectory.hpp"
#include "util.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <boost/bind.hpp>
#include <execinfo.h>
#include <signal.h>


using namespace hebi;

/*
class Roscore_Node {

  public:

  ros::NodeHandle n;
  std::map<std::string, ros::ServiceServer> services;

  Lookup lookup;
  std::map<std::string, std::shared_ptr<Group>> groups;

  Roscore_Node (int argc, char **argv) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    services["/hebicore/add_group_from_names"] = n.advertiseService(
      "/hebicore/add_group_from_names", &Roscore_Node::srv_add_group_from_names, this);

    loop();
  }


  
  bool srv_add_group_from_names(
    add_group_from_names::Request &req, add_group_from_names::Response &res) {
    groups[req.group_name] = lookup.getGroupFromNames(req.families, req.names);

    if (groups[req.group_name]) {

      std::shared_ptr<Group> group = groups[req.group_name];
      group->addFeedbackHandler([](const GroupFeedback& group_fbk) {
        float position = group_fbk[0].actuator().position().get();
        float velocity = group_fbk[0].actuator().velocity().get();
        float effort = group_fbk[0].actuator().effort().get();
        std::cout << "p: " << position << " v: " << velocity << " e: " << effort << std::endl;
      });

      group->setFeedbackFrequencyHz(100);

      return true;
    }
    else {
      return false;
    }
  }
  

  void loop() {
      ros::Rate loop_rate(200);

      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
};
*/



int main(int argc, char **argv) {

  ros::init(argc, argv, "hebicore_node");
  ros::NodeHandle n;

  //Roscore_Node node(argc, argv);

  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({ "HEBI" },
    { "base", "shoulder", "elbow" });

  group->addFeedbackHandler([](const GroupFeedback& group_fbk)
  {
    std::cout << "Got feedback." << std::endl;
    for (int i = 0; i < group_fbk.size(); ++i)
    {
      auto& pos_fbk = group_fbk[i].actuator().position();
      if (pos_fbk)
        std::cout << pos_fbk.get() << std::endl;
      else
        std::cout << "No feedback" << std::endl;
    }
  });

  group->setFeedbackFrequencyHz(25);

  ros::Rate loop_rate(200);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


