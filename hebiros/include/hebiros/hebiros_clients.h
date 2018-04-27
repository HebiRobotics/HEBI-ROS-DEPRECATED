#ifndef HEBIROS_CLIENTS_H
#define HEBIROS_CLIENTS_H

#include "ros/ros.h"

#include "hebiros/AddGroupFromNamesSrv.h"


class HebirosClients {

  public:

    static std::map<std::string, ros::ServiceClient> clients;

    void registerNodeClients();
    void registerGroupClients(std::string group_name);
    void addGroup(hebiros::AddGroupFromNamesSrv::Request &req);

};

#endif
