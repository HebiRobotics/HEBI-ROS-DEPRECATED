#include "ros/ros.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"

using namespace hebiros;


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_01_lookup_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "my_group";

  //Create a client which uses the service to see the entry list of modules
  ros::ServiceClient entry_list_client = n.serviceClient<EntryListSrv>(
    "/hebiros/entry_list");

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  //Create a client which uses the service to find the size of a group
  ros::ServiceClient size_client = n.serviceClient<SizeSrv>(
    "/hebiros/"+group_name+"/size");

  EntryListSrv entry_list_srv;
  AddGroupFromNamesSrv add_group_srv;
  SizeSrv size_srv;

  //Call the entry_list service, displaying each module on the network
  //entry_list_srv.response.entry_list will now be populated with those modules
  entry_list_client.call(entry_list_srv);

  //Construct a group using 3 known modules
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"base", "shoulder", "elbow"};
  add_group_srv.request.families = {"HEBI"};
  //Call the add_group_from_names service to create a group
  //Specific topics and services will now be available under this group's namespace
  add_group_client.call(add_group_srv);

  //Call the size service for the newly created group
  size_client.call(size_srv);
  ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);

  //Spin
  //The group will exist until the node is shutdown
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}









