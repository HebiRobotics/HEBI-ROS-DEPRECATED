#ifndef HEBIROS_SERVICES_H
#define HEBIROS_SERVICES_H

#include "ros/ros.h"
#include "urdf/model.h"

#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/AddGroupFromURDFSrv.h"
#include "hebiros/SizeSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include "hebiros/SetCommandLifetimeSrv.h"
#include "hebiros/SendCommandWithAcknowledgementSrv.h"

using namespace hebiros;


class HebirosServices {

  public:

    static std::map<std::string, ros::ServiceServer> services;

    virtual void registerNodeServices() {}

    virtual void registerGroupServices(std::string group_name) {}

    bool entryList(
      EntryListSrv::Request &req, EntryListSrv::Response &res);

    bool addGroupFromNames(
      AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res);

    bool addGroupFromURDF(
      AddGroupFromURDFSrv::Request &req, AddGroupFromURDFSrv::Response &res);

    bool size(
      SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name);

    bool setFeedbackFrequency(
      SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res,
      std::string group_name);

    bool setCommandLifetime(
      SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
      std::string group_name);

    bool sendCommandWithAcknowledgement(
      SendCommandWithAcknowledgementSrv::Request &req, 
      SendCommandWithAcknowledgementSrv::Response &res, std::string group_name);

    bool split(const std::string &orig, std::string &name, std::string &family);

    void addJointChildren(std::set<std::string>& names, std::set<std::string>& families, 
      std::set<std::string>& full_names, const urdf::Link* link);

};

#endif
