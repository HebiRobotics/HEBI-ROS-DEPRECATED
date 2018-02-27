#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
#include "tf/transform_broadcaster.h"
#include "urdf/model.h"
#include "actionlib/server/simple_action_server.h"

#include "hebiros/EntryMsg.h"
#include "hebiros/EntryListMsg.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/WaypointMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/SettingsMsg.h"
#include "hebiros/PidGainsMsg.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/AddGroupFromUrdfSrv.h"
#include "hebiros/SizeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include "hebiros/SetCommandLifetimeSrv.h"
#include "hebiros/SendCommandWithAcknowledgementSrv.h"
#include "hebiros/TrajectoryAction.h"

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
#include <mutex>
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
    std::map<std::string, ros::ServiceClient> clients;

    std::map<std::string, std::shared_ptr<actionlib::SimpleActionServer<TrajectoryAction>>>   
      trajectory_actions;

    Lookup lookup;
    std::shared_ptr<Lookup::EntryList> entry_list;
    std::map<std::string, std::shared_ptr<Group>> groups;
    std::map<std::string, GroupInfo*> group_infos;
    std::map<std::string, std::map<std::string, int>> group_joints;
    std::map<std::string, sensor_msgs::JointState> group_joint_states;
    std::map<std::string, sensor_msgs::JointState> gazebo_joint_states;
    std::mutex gazebo_joint_states_mutex;

    int node_frequency;
    int action_frequency;
    int feedback_frequency;
    int command_lifetime;

    bool use_gazebo;

    enum class control_strategies {
      CONTROL_STRATEGY_OFF = 0,
      CONTROL_STRATEGY_DIRECT_PWM = 1,
      CONTROL_STRATEGY_2 = 2,
      CONTROL_STRATEGY_3 = 3,
      CONTROL_STRATEGY_4 = 4
    } control_strategies;

    Hebiros_Node (int argc, char **argv);
    ~Hebiros_Node() noexcept(false) {}

    /* Service callback functions*/
    bool srv_entry_list(
      EntryListSrv::Request &req, EntryListSrv::Response &res);

    bool srv_add_group_from_names(
      AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res);

    bool srv_add_group_from_urdf(
      AddGroupFromUrdfSrv::Request &req, AddGroupFromUrdfSrv::Response &res);

    bool srv_size(
      SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name);

    bool srv_set_feedback_frequency(
      SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res,
      std::string group_name);

    bool srv_set_command_lifetime(
      SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
      std::string group_name);

    bool srv_send_command_with_acknowledgement(
      SendCommandWithAcknowledgementSrv::Request &req, 
      SendCommandWithAcknowledgementSrv::Response &res, std::string group_name);

    /* Subscriber callback functions */
    void sub_command(const boost::shared_ptr<CommandMsg const> data,
      std::string group_name);

    void sub_joint_command(const boost::shared_ptr<sensor_msgs::JointState const> data,
      std::string group_name);

    void sub_publish_group_gazebo(
      const boost::shared_ptr<sensor_msgs::JointState const> data,
      std::string group_name, std::string joint_name);

    /* Action execution functions */
    void action_trajectory(const TrajectoryGoalConstPtr& goal, std::string group_name);

    /* Individual group functions */
    void register_group(std::string group_name);

    void publish_group(std::string group_name, const GroupFeedback& group_fbk);

    void unregister_group(std::string group_name);

    /* General */
    bool names_in_order(CommandMsg command_msg);

    bool joint_found(std::string group_name, std::string joint_name);

    void joint_not_found(std::string joint_name);

    void add_joint_command(GroupCommand* group_command,
      sensor_msgs::JointState data, std::string group_name);

    void add_settings_command(GroupCommand* group_command,
      SettingsMsg data, std::string group_name);

    void add_position_gains_command(GroupCommand* group_command,
      PidGainsMsg data, std::string group_name);

    void add_velocity_gains_command(GroupCommand* group_command,
      PidGainsMsg data, std::string group_name);

    void add_effort_gains_command(GroupCommand* group_command,
      PidGainsMsg data, std::string group_name);

    bool split(const std::string &orig, std::string &name, std::string &family);

    void add_joint_children(std::set<std::string>& names, std::set<std::string>& families, 
      std::set<std::string>& full_names, const urdf::Link* link);

    void cleanup();

    void loop();
};

