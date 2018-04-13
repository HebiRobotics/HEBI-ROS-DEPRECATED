#include "hebiros_services_gazebo.h"

#include "hebiros.h"


void HebirosServicesGazebo::registerNodeServices() {

  services["/hebiros/entry_list"] = HebirosNode::n_ptr->advertiseService(
    "/hebiros/entry_list", &HebirosServicesGazebo::entryList, this);

  services["/hebiros/add_group_from_names"] = HebirosNode::n_ptr->advertiseService(
    "/hebiros/add_group_from_names", &HebirosServicesGazebo::addGroupFromNames, this);

  services["/hebiros/add_group_from_urdf"] = HebirosNode::n_ptr->advertiseService(
    "/hebiros/add_group_from_urdf", &HebirosServicesGazebo::addGroupFromURDF, this);
}

void HebirosServicesGazebo::registerGroupServices(std::string group_name) {

  services["/hebiros/"+group_name+"/size"] =
    HebirosNode::n_ptr->advertiseService<SizeSrv::Request, SizeSrv::Response>(
    "/hebiros/"+group_name+"/size",
    boost::bind(&HebirosServicesGazebo::size, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/set_feedback_frequency"] =
    HebirosNode::n_ptr->advertiseService<SetFeedbackFrequencySrv::Request,
    SetFeedbackFrequencySrv::Response>(
    "/hebiros/"+group_name+"/set_feedback_frequency",
    boost::bind(&HebirosServicesGazebo::setFeedbackFrequency, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/set_command_lifetime"] =
    HebirosNode::n_ptr->advertiseService<SetCommandLifetimeSrv::Request,
    SetCommandLifetimeSrv::Response>(
    "/hebiros/"+group_name+"/set_command_lifetime",
    boost::bind(&HebirosServicesGazebo::setCommandLifetime, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/send_command_with_acknowledgement"] =
    HebirosNode::n_ptr->advertiseService<SendCommandWithAcknowledgementSrv::Request,
    SendCommandWithAcknowledgementSrv::Response>(
    "/hebiros/"+group_name+"/send_command_with_acknowledgement",
    boost::bind(&HebirosServicesGazebo::sendCommandWithAcknowledgement, this, _1, _2, group_name));
}

bool HebirosServicesGazebo::entryList(
  EntryListSrv::Request &req, EntryListSrv::Response &res) {

  return true;
}

bool HebirosServicesGazebo::addGroupFromNames(
  AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res) {

  if (HebirosGroup::findGroup(req.group_name)) {

    ROS_WARN("Group [%s] already exists", req.group_name.c_str());
    return true;
  }
  
  HebirosGroupGazebo group(req.group_name);

  if (!HebirosServices::addGroupFromNames(req, res)) {
    HebirosGroupGazebo::removeGroup(req.group_name);
    return false;
  }

  registerGroupServices(req.group_name);

  return true;
}

bool HebirosServicesGazebo::addGroupFromURDF(
  AddGroupFromURDFSrv::Request &req, AddGroupFromURDFSrv::Response &res) {

  std::string urdf_name("robot_description");
  urdf::Model urdf_model;
  if (!urdf_model.initParam(urdf_name))
  {
    ROS_WARN("Could not load robot_description");
    return false;
  }

  std::set<std::string> joint_names;
  std::set<std::string> family_names;
  std::set<std::string> joint_full_names;
  HebirosServices::addJointChildren(
    joint_names, family_names, joint_full_names, urdf_model.getRoot().get());

  AddGroupFromNamesSrv::Request names_req;
  AddGroupFromNamesSrv::Response names_res;
  names_req.group_name = req.group_name;
  names_req.families.insert(names_req.families.end(), family_names.begin(), family_names.end());
  names_req.names.insert(names_req.names.end(), joint_names.begin(), joint_names.end());

  return HebirosNode::services_gazebo.addGroupFromNames(names_req, names_res);
}

bool HebirosServicesGazebo::size(
  SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name) {

  HebirosServices::size(req, res, group_name);

  return true;
}

bool HebirosServicesGazebo::setFeedbackFrequency(
  SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res,
  std::string group_name) {

  return true;
}

bool HebirosServicesGazebo::setCommandLifetime(
  SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
  std::string group_name) {

  return true;
}

bool HebirosServicesGazebo::sendCommandWithAcknowledgement(
  SendCommandWithAcknowledgementSrv::Request &req, 
  SendCommandWithAcknowledgementSrv::Response &res, std::string group_name) {

  return true;
}



