#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
//#include "urdf/model.h"

#include "hebi/HebiEntryMsg.h"
#include "hebi/HebiEntryListMsg.h"
#include "hebi/HebiFeedbackMsg.h"
#include "hebi/entry_list_srv.h"
#include "hebi/add_group_from_names_srv.h"
#include "hebi/size_srv.h"
#include "hebi/set_feedback_frequency_srv.h"
#include "hebi/set_command_lifetime_srv.h"

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


class Hebiros_Node {

  public:

    ~Hebiros_Node() noexcept(false) {}

    ros::NodeHandle n;
    std::map<std::string, ros::Publisher> publishers;
    std::map<std::string, ros::Subscriber> subscribers;
    std::map<std::string, ros::ServiceServer> services;

    Lookup lookup;
    std::shared_ptr<Lookup::EntryList> entry_list;
    std::map<std::string, std::shared_ptr<Group>> groups;
    std::map<std::string, GroupInfo*> group_infos;

    int feedback_frequency;
    int command_lifetime;

    Hebiros_Node (int argc, char **argv) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      services["/hebicore/entry_list"] = n.advertiseService(
        "/hebicore/entry_list", &Hebiros_Node::srv_entry_list, this);
      
      services["/hebicore/add_group_from_names"] = n.advertiseService(
        "/hebicore/add_group_from_names", &Hebiros_Node::srv_add_group_from_names, this);
      
      n.param<int>("/hebicore/feedback_frequency", feedback_frequency, 100);
      
      n.param<int>("/hebicore/command_lifetime", command_lifetime, 100);

      loop();
    }

    bool srv_entry_list(
      entry_list_srv::Request &req, entry_list_srv::Response &res) {
      HebiEntryListMsg entry_list_msg;
      HebiEntryMsg entry_msg;

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
      add_group_from_names_srv::Request &req, add_group_from_names_srv::Response &res) {
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
      size_srv::Request &req, size_srv::Response &res, std::string group_name) {
      std::shared_ptr<Group> group = groups[group_name];

      if (group) {
        res.size = group->size();
        return true;
      }
      else {
        return false;
      }
    }

    bool srv_set_feedback_frequency(
      set_feedback_frequency_srv::Request &req, set_feedback_frequency_srv::Response &res,
      std::string group_name) {
      std::shared_ptr<Group> group = groups[group_name];

      group->setFeedbackFrequencyHz(req.feedback_frequency);

      return true;
    }

    bool srv_set_command_lifetime(
      set_command_lifetime_srv::Request &req, set_command_lifetime_srv::Response &res,
      std::string group_name) {
      std::shared_ptr<Group> group = groups[group_name];

      group->setCommandLifetimeMs(req.command_lifetime);

      return true;
    }

    void register_group(std::string group_name) {

      publishers["/hebicore/"+group_name+"/feedback"] =
        n.advertise<HebiFeedbackMsg>("/hebicore/"+group_name+"/feedback", 100);

      publishers["/hebicore/"+group_name+"/feedback/joint_state"] =
        n.advertise<sensor_msgs::JointState>("/hebicore/"+group_name+"/feedback/joint_state", 100);

      subscribers["/hebicore/"+group_name+"/command/joint_state"] = 
        n.subscribe<sensor_msgs::JointState>("/hebicore/"+group_name+"/command/joint_state", 100,
        boost::bind(&Hebiros_Node::sub_command, this, _1, group_name));

      services["/hebicore/"+group_name+"/size"] =
        n.advertiseService<size_srv::Request, size_srv::Response>("/hebicore/"+group_name+"/size",
        boost::bind(&Hebiros_Node::srv_size, this, _1, _2, group_name));

      services["/hebicore/"+group_name+"/set_feedback_frequency"] =
        n.advertiseService<set_feedback_frequency_srv::Request,
        set_feedback_frequency_srv::Response>(
        "/hebicore/"+group_name+"/set_feedback_frequency",
        boost::bind(&Hebiros_Node::srv_set_feedback_frequency, this, _1, _2, group_name));

      services["/hebicore/"+group_name+"/set_command_lifetime"] =
        n.advertiseService<set_command_lifetime_srv::Request,
        set_command_lifetime_srv::Response>(
        "/hebicore/"+group_name+"/set_command_lifetime",
        boost::bind(&Hebiros_Node::srv_set_command_lifetime, this, _1, _2, group_name));

      std::shared_ptr<Group> group = groups[group_name];
      group_infos[group_name] = new GroupInfo(group->size());
      group->requestInfo(group_infos[group_name]);

      group->addFeedbackHandler([this, group_name, group](const GroupFeedback& group_fbk) {
        this->publish_group(group_name, group, group_fbk);
      });

      group->setFeedbackFrequencyHz(feedback_frequency);
      group->setCommandLifetimeMs(command_lifetime);
    }

    void publish_group(std::string group_name, std::shared_ptr<Group> group,
      const GroupFeedback& group_fbk) {

      HebiFeedbackMsg feedback_msg;
      sensor_msgs::JointState joint_state_msg;

      sensor_msgs::Imu imu_msg;
      imu_msg.orientation_covariance[0] = -1;
      imu_msg.angular_velocity_covariance[0] = -1;
      imu_msg.linear_acceleration_covariance[0] = -1;

      for (int i = 0; i < group->size(); i++) {
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

      publishers["/hebicore/"+group_name+"/feedback"].publish(feedback_msg);
      publishers["/hebicore/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
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
  Hebiros_Node node(argc, argv);

  return 0;
}

