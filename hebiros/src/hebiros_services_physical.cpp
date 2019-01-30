#include "hebiros_services_physical.h"
#include "hebiros.h"
#include "hebiros_group_registry.h"

#include <memory>

void HebirosServicesPhysical::registerNodeServices() {

  services["hebiros/entry_list"] = HebirosNode::n_ptr->advertiseService(
    "hebiros/entry_list", &HebirosServicesPhysical::entryList, this);

  services["hebiros/add_group_from_names"] = HebirosNode::n_ptr->advertiseService(
    "hebiros/add_group_from_names", &HebirosServicesPhysical::addGroupFromNames, this);

  services["hebiros/add_group_from_urdf"] = HebirosNode::n_ptr->advertiseService(
    "hebiros/add_group_from_urdf", &HebirosServicesPhysical::addGroupFromURDF, this);

  services["hebiros/add_model_from_urdf"] = HebirosNode::n_ptr->advertiseService(
    "hebiros/add_model_from_urdf", &HebirosServicesPhysical::addModelFromURDF, this);
}

void HebirosServicesPhysical::registerGroupServices(std::string group_name) {

  services["hebiros/"+group_name+"/size"] =
    HebirosNode::n_ptr->advertiseService<SizeSrv::Request, SizeSrv::Response>(
    "hebiros/"+group_name+"/size",
    boost::bind(&HebirosServicesPhysical::size, this, _1, _2, group_name));

  services["hebiros/"+group_name+"/set_feedback_frequency"] =
    HebirosNode::n_ptr->advertiseService<SetFeedbackFrequencySrv::Request,
    SetFeedbackFrequencySrv::Response>(
    "hebiros/"+group_name+"/set_feedback_frequency",
    boost::bind(&HebirosServicesPhysical::setFeedbackFrequency, this, _1, _2, group_name));

  services["hebiros/"+group_name+"/set_command_lifetime"] =
    HebirosNode::n_ptr->advertiseService<SetCommandLifetimeSrv::Request,
    SetCommandLifetimeSrv::Response>(
    "hebiros/"+group_name+"/set_command_lifetime",
    boost::bind(&HebirosServicesPhysical::setCommandLifetime, this, _1, _2, group_name));

  services["hebiros/"+group_name+"/send_command_with_acknowledgement"] =
    HebirosNode::n_ptr->advertiseService<SendCommandWithAcknowledgementSrv::Request,
    SendCommandWithAcknowledgementSrv::Response>(
    "hebiros/"+group_name+"/send_command_with_acknowledgement",
    boost::bind(&HebirosServicesPhysical::sendCommandWithAcknowledgement, this, _1, _2, group_name));
}

bool HebirosServicesPhysical::entryList(
  EntryListSrv::Request &req, EntryListSrv::Response &res) {

  EntryListMsg entry_list_msg;
  EntryMsg entry_msg;

  std::shared_ptr<Lookup::EntryList> entry_list = lookup.getEntryList();
  entry_list_msg.size = entry_list->size();

  ROS_INFO("Entry list:");
  for (auto entry : *entry_list) {
    entry_msg.name = entry.name_;
    entry_msg.family = entry.family_;
    // TODO: add entry.mac_address_ here! (it is now properly filled in)
    entry_msg.mac_address = 0;

    ROS_INFO("/%s/%s/%2x:%2x:%2x:%2x:%2x:%2x", entry_msg.family.c_str(), entry_msg.name.c_str(),
      entry.mac_address_[0], entry.mac_address_[1], entry.mac_address_[2],
      entry.mac_address_[3], entry.mac_address_[4], entry.mac_address_[5]);
    entry_list_msg.entries.push_back(entry_msg);
  }

  res.entry_list = entry_list_msg;
  return true;
}

bool HebirosServicesPhysical::addGroup(
  AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res,
  std::map<std::string, std::string> joint_full_names) {

  auto& registry = HebirosGroupRegistry::Instance();

  if (registry.hasGroup(req.group_name)) {

    ROS_WARN("Group [%s] already exists", req.group_name.c_str());
    return true;
  }

  std::shared_ptr<hebi::Group> group_ptr = lookup.getGroupFromNames(req.families, req.names);

  if (!group_ptr) {

    ROS_WARN("Lookup of group [%s] failed", req.group_name.c_str());
    return false;
  }

  std::unique_ptr<HebirosGroupPhysical> group(new HebirosGroupPhysical(group_ptr));
  group->joint_full_names = joint_full_names;

  if (!HebirosServices::addGroup(req, res, joint_full_names, std::move(group))) {
    return false;
  }

  registerGroupServices(req.group_name);
  HebirosNode::publishers_physical.registerGroupPublishers(req.group_name);
  HebirosNode::subscribers_physical.registerGroupSubscribers(req.group_name);
  HebirosNode::actions.registerGroupActions(req.group_name);

  return true;
}

bool HebirosServicesPhysical::addGroupFromNames(
  AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res) {
  
  std::map<std::string, std::string> joint_full_names;

  return HebirosNode::services_physical.addGroup(req, res, joint_full_names);
}

bool HebirosServicesPhysical::addGroupFromURDF(
  AddGroupFromURDFSrv::Request &req, AddGroupFromURDFSrv::Response &res) {

  std::string urdf_name("robot_description");
  urdf::Model urdf_model;
  if (!urdf_model.initParam(urdf_name))
  {
    ROS_WARN("Could not load robot_description");
    return false;
  }
  else {
    HebirosServices::addGroupFromURDF(req, res);
  }

  std::set<std::string> joint_names;
  std::set<std::string> family_names;
  std::map<std::string, std::string> joint_full_names;

  HebirosServices::addJointChildren(joint_names, family_names, joint_full_names,
    urdf_model.getRoot().get());

  AddGroupFromNamesSrv::Request names_req;
  AddGroupFromNamesSrv::Response names_res;
  names_req.group_name = req.group_name;
  names_req.families.insert(names_req.families.end(), family_names.begin(), family_names.end());
  names_req.names.insert(names_req.names.end(), joint_names.begin(), joint_names.end());

  return HebirosNode::services_physical.addGroup(names_req, names_res, joint_full_names);
}

bool HebirosServicesPhysical::addModelFromURDF(
  AddModelFromURDFSrv::Request &req, AddModelFromURDFSrv::Response &res) {

  return HebirosServices::addModelFromURDF(req, res);
}

bool HebirosServicesPhysical::size(
  SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name) {

  HebirosServices::size(req, res, group_name);

  return true;
}

// TODO: move this function to HebirosServices, not Physical
bool HebirosServicesPhysical::setFeedbackFrequency(
  SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res,
  std::string group_name) {

  auto& registry = HebirosGroupRegistry::Instance();

  HebirosGroup* group = registry.getGroup(group_name);
  if (!group)
    return false;
  group->setFeedbackFrequency(req.feedback_frequency);

  HebirosServices::setFeedbackFrequency(req, res, group_name);

  return true;
}

// TODO: move this function to HebirosServices, not Physical
bool HebirosServicesPhysical::setCommandLifetime(
  SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
  std::string group_name) {

  auto& registry = HebirosGroupRegistry::Instance();

  HebirosGroup* group = registry.getGroup(group_name);
  if (!group)
    return false;
  group->setCommandLifetime(req.command_lifetime);

  HebirosServices::setCommandLifetime(req, res, group_name);

  return true;
}

bool HebirosServicesPhysical::sendCommandWithAcknowledgement(
  SendCommandWithAcknowledgementSrv::Request &req, 
  SendCommandWithAcknowledgementSrv::Response &res, std::string group_name) {

  // TODO: replace with better abstraction later -- move send command into group
  HebirosGroupPhysical* group = dynamic_cast<HebirosGroupPhysical*>
    (hebiros::HebirosGroupRegistry::Instance().getGroup(group_name)); 
  if (!group) {
    ROS_WARN("Improper group type during command call");
    return false;
  }

  GroupCommand group_command(group->size);

  sensor_msgs::JointState joint_data;
  joint_data.name = req.command.name;
  joint_data.position = req.command.position;
  joint_data.velocity = req.command.velocity;
  joint_data.effort = req.command.effort;
  SettingsMsg settings_data;
  settings_data = req.command.settings;

  HebirosSubscribersPhysical::addJointCommand(&group_command, joint_data, group_name);
  HebirosSubscribersPhysical::addSettingsCommand(&group_command, settings_data, group_name);

  return group->group_ptr->sendCommandWithAcknowledgement(group_command);

  return true;
}



