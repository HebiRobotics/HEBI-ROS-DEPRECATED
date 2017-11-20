#include "ros/ros.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromUrdfSrv.h"

using namespace hebiros;


//Global variable and callback function used to store feedback data
FeedbackMsg feedback;

void feedback_callback(FeedbackMsg data) {
  feedback = data;
}


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_rviz_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "x_demo";

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromUrdfSrv>(
    "/hebiros/add_group_from_urdf");

  //Create a subscriber to receive feedback from a group
  //Register feedback_callback as a callback which runs when feedback is received
  ros::Subscriber feedback_subscriber = n.subscribe(
    "/hebiros/"+group_name+"/feedback", 100, feedback_callback);

  AddGroupFromUrdfSrv add_group_srv;

  //Construct a group using a urdf
  add_group_srv.request.group_name = group_name;
  //Call the add_group_from_urdf service to create a group
  //Specific topics and services will now be available under this group's namespace
  add_group_client.call(add_group_srv);

  return 0;
}









