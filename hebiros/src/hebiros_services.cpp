#include "hebiros_services.h"

#include "hebiros.h"

#include "hebiros_group.h"
#include "hebiros_group_registry.h"

std::map<std::string, ros::ServiceServer> HebirosServices::services;

void HebirosServices::registerModelServices(const std::string& model_name) {
  services["hebiros/"+model_name+"/fk"] =
    HebirosNode::n_ptr->advertiseService<ModelFkSrv::Request, ModelFkSrv::Response>(
    "hebiros/"+model_name+"/fk",
    boost::bind(&HebirosServices::fk, this, _1, _2, model_name));
}

bool HebirosServices::entryList(
  EntryListSrv::Request &req, EntryListSrv::Response &res) {

  return true;
}

bool HebirosServices::addGroup(
  AddGroupFromNamesSrv::Request &req, AddGroupFromNamesSrv::Response &res,
  const std::map<std::string, std::string>& joint_full_names, std::unique_ptr<HebirosGroup> group_tmp) {

  if (req.families.size() != 1 && req.families.size() != req.names.size()) {
    ROS_WARN("Invalid number of familes for group [%s]", req.group_name.c_str());
    return false;
  }

  auto& registry = HebirosGroupRegistry::Instance();

  if (registry.hasGroup(req.group_name))
  {
    ROS_WARN("Group [%s] already exists!", req.group_name.c_str());
    return false;
  }

  registry.addGroup(req.group_name, std::move(group_tmp));
  HebirosGroup* group = registry.getGroup(req.group_name);

  ROS_INFO("Created group [%s]:", req.group_name.c_str());
  for (int i = 0; i < req.families.size(); i++) {
    for (int j = 0; j < req.names.size(); j++) {

      if ((req.families.size() == 1) ||
        (req.families.size() == req.names.size() && i == j)) {

        std::string joint_name = req.families[i]+"/"+req.names[j];
        ROS_INFO("/%s/%s", req.group_name.c_str(), joint_name.c_str());

        group->joints[joint_name] = j;
      }
    }
  }

  group->joint_full_names = joint_full_names;
  group->size = group->joints.size();

  return true;
}

bool HebirosServices::addGroupFromURDF(
  AddGroupFromURDFSrv::Request &req, AddGroupFromURDFSrv::Response &res) {

  ROS_INFO("Loaded URDF from robot_description");

  return true;
}

bool HebirosServices::split(const std::string &orig, std::string &name, std::string &family) {
  std::stringstream ss(orig);

  if (!std::getline(ss, family, '/')) {
    return false;
  }
  if (!std::getline(ss, name, '/')) {
    return false;
  }

  return true;
}

void HebirosServices::addJointChildren(std::set<std::string>& names,
  std::set<std::string>& families, std::map<std::string, std::string>& full_names,
  const urdf::Link* link) {
  for (auto& joint : link->child_joints) {

    if (joint->type != urdf::Joint::FIXED) {
      std::string name, family;

      if (split(joint->name, name, family)) {
        names.insert(name);
        families.insert(family);
        full_names[family+'/'+name] = joint->name;
      }
    }
  }

  for (auto& link_child : link->child_links) {
    addJointChildren(names, families, full_names, link_child.get());
  }
}

bool HebirosServices::addModelFromURDF(
      AddModelFromURDFSrv::Request &req, AddModelFromURDFSrv::Response &res) {
  // Default to "robot_description" if not given.
  static const std::string default_desc("robot_description");
  auto& description_param = req.description_param.size() == 0 ? default_desc : req.description_param;

  if (HebirosModel::load(req.model_name, req.description_param)) {
    registerModelServices(req.model_name);
    return true;
  }
  return false;
}

bool HebirosServices::size(
  SizeSrv::Request &req, SizeSrv::Response &res, std::string group_name) {

  auto& registry = HebirosGroupRegistry::Instance();

  if (!registry.hasGroup(group_name))
  {
    res.size = -1;
    ROS_INFO("hebiros/%s not found; could not get size", group_name.c_str());
    return false;
  }

  res.size = registry.getGroup(group_name)->size;

  ROS_INFO("hebiros/%s size=%d", group_name.c_str(), res.size);

  return true;
}

bool HebirosServices::setFeedbackFrequency(
  SetFeedbackFrequencySrv::Request &req, SetFeedbackFrequencySrv::Response &res,
  std::string group_name) {

  ROS_INFO("hebiros/%s feedback_frequency=%d", group_name.c_str(), req.feedback_frequency);

  return true;
}

bool HebirosServices::setCommandLifetime(
  SetCommandLifetimeSrv::Request &req, SetCommandLifetimeSrv::Response &res,
  std::string group_name) {

  ROS_INFO("hebiros/%s command_lifetime=%d", group_name.c_str(), req.command_lifetime);

  return true;
}

bool HebirosServices::sendCommandWithAcknowledgement(
  SendCommandWithAcknowledgementSrv::Request &req, 
  SendCommandWithAcknowledgementSrv::Response &res, std::string group_name) {

  return true;
}

bool HebirosServices::fk(ModelFkSrv::Request& req, ModelFkSrv::Response& res, const std::string& model_name) {
  // Look up model
  auto model = HebirosModel::getModel(model_name);
  if (!model)
    return false;

  // Convert ROS message into HEBI types
  HebiFrameType frame_type = HebiFrameTypeOutput;
  if (req.frame_type == ModelFkSrv::Request::FrameTypeCenterOfMass)
    frame_type == HebiFrameTypeCenterOfMass;
  else if (req.frame_type == ModelFkSrv::Request::FrameTypeOutput)
    frame_type == HebiFrameTypeOutput;
  else
    return false; // Invalid frame type!
  Eigen::VectorXd joints(req.positions.size());
  for (size_t i = 0; i < joints.size(); ++i)
    joints[i] = req.positions[i];
 
  // Get the frames from the robot model 
  hebi::robot_model::Matrix4dVector frames;
  model->getModel().getFK(HebiFrameTypeOutput, joints, frames);

  // Fill in the ROS messages
  res.frames.resize(frames.size() * 16);
  for (size_t i = 0; i < frames.size(); ++i) {
    for (size_t j = 0; j < 4; ++j) {
      for (size_t k = 0; k < 4; ++k) {
        res.frames[i * 16 + j * 4 + k] = frames[i](j, k);
      }
    }
  }

  return true;
}
