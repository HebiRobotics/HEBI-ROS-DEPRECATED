#ifndef HEBIROS_CLIENTS_H
#define HEBIROS_CLIENTS_H

#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SetCommandLifetimeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"


class HebirosClients {

  public:

    static std::map<std::string, ros::ServiceClient> clients;

    void registerNodeClients();
    void registerGroupClients(std::string group_name);
    bool addGroup(hebiros::AddGroupFromNamesSrv::Request &req);
    bool setCommandLifetime(hebiros::SetCommandLifetimeSrv::Request &req,
      std::string group_name);
    bool setFeedbackFrequency(hebiros::SetFeedbackFrequencySrv::Request &req,
      std::string group_name);
    bool acknowledge(std_srvs::Empty::Request &req, std::string group_name);

};

#endif
