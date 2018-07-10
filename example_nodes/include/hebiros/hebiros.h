#ifndef HEBIROS_H
#define HEBIROS_H

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
#include "hebiros/AddGroupFromURDFSrv.h"
#include "hebiros/AddModelFromURDFSrv.h"
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
#include "robot_model.hpp"
#include "log_file.hpp"
#include "lookup.hpp"
#include "mac_address.hpp"
#include "trajectory.hpp"
#include "util.hpp"

#include "hebiros_services.h"
#include "hebiros_services_gazebo.h"
#include "hebiros_services_physical.h"
#include "hebiros_subscribers.h"
#include "hebiros_subscribers_gazebo.h"
#include "hebiros_subscribers_physical.h"
#include "hebiros_publishers.h"
#include "hebiros_publishers_gazebo.h"
#include "hebiros_publishers_physical.h"
#include "hebiros_clients.h"
#include "hebiros_actions.h"
#include "hebiros_group.h"
#include "hebiros_group_gazebo.h"
#include "hebiros_group_physical.h"
#include "hebiros_parameters.h"
#include "hebiros_model.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <boost/bind.hpp>
#include <execinfo.h>
#include <signal.h>

using namespace hebi;
using namespace hebiros;

class HebirosNode {

  public:

    ros::NodeHandle n;
    static std::shared_ptr<ros::NodeHandle> n_ptr;
    static HebirosPublishersGazebo publishers_gazebo;
    static HebirosPublishersPhysical publishers_physical;
    static HebirosSubscribersGazebo subscribers_gazebo;
    static HebirosSubscribersPhysical subscribers_physical;
    static HebirosServicesGazebo services_gazebo;
    static HebirosServicesPhysical services_physical;
    static HebirosClients clients;
    static HebirosActions actions;

    bool use_gazebo;

    HebirosNode(int argc, char **argv);

    void cleanup();

    void loop();
};

#endif

