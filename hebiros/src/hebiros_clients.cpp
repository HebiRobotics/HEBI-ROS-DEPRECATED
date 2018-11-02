#include "hebiros_clients.h"

#include "hebiros.h"

using namespace hebiros;


std::map<std::string, ros::ServiceClient> HebirosClients::clients;

void HebirosClients::registerNodeClients() {

  clients["hebiros_gazebo_plugin/add_group"] =
    HebirosNode::n_ptr->serviceClient<AddGroupFromNamesSrv>("hebiros_gazebo_plugin/add_group");
}

void HebirosClients::registerGroupClients(std::string group_name) {

  clients["hebiros_gazebo_plugin/set_command_lifetime/"+group_name] =
    HebirosNode::n_ptr->serviceClient<SetCommandLifetimeSrv>(
    "hebiros_gazebo_plugin/set_command_lifetime/"+group_name);

  clients["hebiros_gazebo_plugin/set_feedback_frequency/"+group_name] =
    HebirosNode::n_ptr->serviceClient<SetFeedbackFrequencySrv>(
    "hebiros_gazebo_plugin/set_feedback_frequency/"+group_name);

  clients["hebiros_gazebo_plugin/acknowledge/"+group_name] =
    HebirosNode::n_ptr->serviceClient<std_srvs::Empty>(
    "hebiros_gazebo_plugin/acknowledge/"+group_name);
}

bool HebirosClients::addGroup(AddGroupFromNamesSrv::Request &req) {

  AddGroupFromNamesSrv srv;
  srv.request = req;
  return clients["hebiros_gazebo_plugin/add_group"].call(srv);
}

bool HebirosClients::setCommandLifetime(SetCommandLifetimeSrv::Request &req,
  std::string group_name) {

  SetCommandLifetimeSrv srv;
  srv.request = req;
  return clients["hebiros_gazebo_plugin/set_feedback_frequency/"+group_name].call(srv);
}

bool HebirosClients::setFeedbackFrequency(SetFeedbackFrequencySrv::Request &req,
  std::string group_name) {

  SetFeedbackFrequencySrv srv;
  srv.request = req;
  return clients["hebiros_gazebo_plugin/set_feedback_frequency/"+group_name].call(srv);
}

bool HebirosClients::acknowledge(std_srvs::Empty::Request &req,
  std::string group_name) {

  std_srvs::Empty srv;
  srv.request = req;
  return clients["hebiros_gazebo_plugin/acknowledge/"+group_name].call(srv);
}
