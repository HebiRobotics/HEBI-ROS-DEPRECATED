#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"

#include "hebiros/EntryMsg.h"
#include "hebiros/EntryListMsg.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include "hebiros/SetCommandLifetimeSrv.h"

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
using namespace hebiros;

class Hebiros_Node {

  public:

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

    Hebiros_Node (int argc, char **argv);
    ~Hebiros_Node() noexcept(false) {}

    /* Service callback functions*/
    bool srv_entry_list(
      EntryListSrv::Request &req, EntryListSrv::Response &res);

    bool srv_add_group_from_names(
      AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res);

    bool srv_size(
      SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name);

    bool srv_set_feedback_frequency(
      SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res,
      std::string group_name);

    bool srv_set_command_lifetime(
      SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
      std::string group_name);

    /* Subscriber callback functions */
    void sub_command(const boost::shared_ptr<sensor_msgs::JointState const> data,
      std::string group_name);

    /* Individual group functions */
    void register_group(std::string group_name);

    void publish_group(std::string group_name, const GroupFeedback& group_fbk);

    void unregister_group(std::string group_name);

    /* General */
    void cleanup();

    void loop();
};

