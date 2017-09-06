#include "ros/ros.h"
//#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
//#include <urdf/model.h>

#include "hebi/Entry.h"
#include "hebi/EntryList.h"
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


#define HEBI_STACK_SIZE 25
static void* stack[HEBI_STACK_SIZE];

static void print_stack() {

  int size;

  // get void*'s for all entries on the stack
  size = backtrace(stack, HEBI_STACK_SIZE);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %p:\n", signal);
  backtrace_symbols_fd(stack, size, STDERR_FILENO);
  //raise(SIGSTOP);
  //exit(1);

}

static void segfault_func(int signal, siginfo_t* si, void* arg) {
  #if 0  
  int size;

  // get void*'s for all entries on the stack
  size = backtrace(stack, HEBI_STACK_SIZE);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", signal);
  backtrace_symbols_fd(stack, size, STDERR_FILENO);
  raise(SIGSTOP);
  exit(1);
  #endif
  print_stack();
}

static void setupHandler() {
  struct sigaction sa;
  memset(&sa, 0, sizeof(struct sigaction));
  sigemptyset(&sa.sa_mask);
  sa.sa_sigaction = segfault_func;
  sa.sa_flags   = SA_SIGINFO;
  sigaction(SIGSEGV, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGQUIT, &sa, NULL);
  sigaction(SIGABRT, &sa, NULL);
  
}


class Roscore_Node {

  public:

    ~Roscore_Node() noexcept(false) {}

    ros::NodeHandle n;
    std::map<std::string, ros::Publisher> publishers;
    std::map<std::string, ros::Subscriber> subscribers;
    std::map<std::string, ros::ServiceServer> services;

    Lookup lookup;
    std::shared_ptr<Lookup::EntryList> entry_list;
    std::map<std::string, std::shared_ptr<Group>> groups;
    std::map<std::string, GroupInfo*> group_infos;

    Roscore_Node (int argc, char **argv) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      services["/hebicore/entry_list"] = n.advertiseService(
        "/hebicore/entry_list", &Roscore_Node::srv_entry_list, this);
      services["/hebicore/add_group_from_names"] = n.advertiseService(
        "/hebicore/add_group_from_names", &Roscore_Node::srv_add_group_from_names, this);

      loop();
    }

    bool srv_entry_list(
      entry_list::Request &req, entry_list::Response &res) {
      EntryList entry_list_msg;
      Entry entry_msg;

      entry_list = lookup.getEntryList();
      entry_list_msg.size = entry_list->size();

      for (int i = 0; i < entry_list->size(); ++i) {
        auto entry = entry_list->getEntry(i);
        entry_msg.name = entry.name_;
        entry_msg.family = entry.family_;
        entry_msg.macaddress = 0;

        entry_list_msg.entries.push_back(entry_msg);
      }

      res.entry_list = entry_list_msg;
      return true;
    }

    bool srv_add_group_from_names(
      add_group_from_names::Request &req, add_group_from_names::Response &res) {
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

    void sub_command(const boost::shared_ptr<sensor_msgs::JointState const> data,
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

    bool srv_size(
      size::Request &req, size::Response &res, std::string group_name) {
      if (groups[group_name]) {
        res.size = groups[group_name]->size();
        return true;
      }
      else {
        return false;
      }
    }

    void register_group(std::string group_name) {

      publishers["/hebicore/"+group_name+"/feedback"] =
        n.advertise<sensor_msgs::JointState>("/hebicore/"+group_name+"/feedback", 100);
      subscribers["/hebicore/"+group_name+"/command"] = 
        n.subscribe<sensor_msgs::JointState>("/hebicore/"+group_name+"/command", 100,
        boost::bind(&Roscore_Node::sub_command, this, _1, group_name));
      services["/hebicore/"+group_name+"/size"] =
        n.advertiseService<size::Request, size::Response>("/hebicore/"+group_name+"/size",
        boost::bind(&Roscore_Node::srv_size, this, _1, _2, group_name));

      std::shared_ptr<Group> group = groups[group_name];
      group_infos[group_name] = new GroupInfo(group->size());
      group->requestInfo(group_infos[group_name]);

      group->addFeedbackHandler([this, group_name, group](const GroupFeedback& group_fbk) {
        this->publish_group(group_name, group, group_fbk);
      });

      group->setFeedbackFrequencyHz(100);
      group->setCommandLifetimeMs(100);
    }

    void publish_group(std::string group_name, std::shared_ptr<Group> group,
      const GroupFeedback& group_fbk) {

      sensor_msgs::JointState feedback_msg;

      for (int i = 0; i < group->size(); i++) {
        std::string name = (*group_infos[group_name])[i].settings().name().get();
        std::string family = (*group_infos[group_name])[i].settings().family().get();
        float position = group_fbk[i].actuator().position().get();
        float velocity = group_fbk[i].actuator().velocity().get();
        float effort = group_fbk[i].actuator().effort().get();

        feedback_msg.name.push_back("/"+family+"/"+name);
        feedback_msg.position.push_back(position);
        feedback_msg.velocity.push_back(velocity);
        feedback_msg.effort.push_back(effort);
      }

      publishers["/hebicore/"+group_name+"/feedback"].publish(feedback_msg);
   } 

    void loop() {
      ros::Rate loop_rate(200);

      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
      }
    }

};


int main(int argc, char **argv) {

  ros::init(argc, argv, "hebicore_node");
  //setupHandler();
  Roscore_Node node(argc, argv);

  return 0;
}
