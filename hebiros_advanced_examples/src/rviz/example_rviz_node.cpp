#include "ros/ros.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromURDFSrv.h"

using namespace hebiros;


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_rviz_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "x_demo";

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromURDFSrv>(
    "/hebiros/add_group_from_urdf");

  AddGroupFromURDFSrv add_group_srv;

  //Construct a group using a urdf
  add_group_srv.request.group_name = group_name;
  //Call the add_group_from_urdf service to create a group until it succeeds
  //Specific topics and services will now be available under this group's namespace
  while(!add_group_client.call(add_group_srv)) {}

  return 0;
}









