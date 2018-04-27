#include "hebiros_clients.h"

#include "hebiros.h"

using namespace hebiros;


std::map<std::string, ros::ServiceClient> HebirosClients::clients;

void HebirosClients::registerNodeClients() {

  clients["/hebiros_gazebo_plugin/add_group"] =
    HebirosNode::n_ptr->serviceClient<AddGroupFromNamesSrv>("/hebiros_gazebo_plugin/add_group");
}

void HebirosClients::registerGroupClients(std::string group_name) {

  //clients["/hebiros_gazebo_plugin/add_group"] =
  //  HebirosNode::n_ptr->serviceClient<AddGroupFromNamesSrv>("/hebiros_gazebo_plugin/add_group");
}

void HebirosClients::addGroup(AddGroupFromNamesSrv::Request &req) {

  AddGroupFromNamesSrv srv;
  srv.request = req;
  clients["/hebiros_gazebo_plugin/add_group"].call(srv);
}
