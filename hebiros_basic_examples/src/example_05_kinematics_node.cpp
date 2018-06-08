//TODO: Finish example
#include "ros/ros.h"
#include "hebiros/AddModelFromURDFSrv.h"

using namespace hebiros;


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_05_kinematics_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string model_name = "my_model";

  ros::ServiceClient add_model_client = n.serviceClient<AddModelFromURDFSrv>(
    "/hebiros/add_model_from_urdf");

  AddModelFromURDFSrv add_model_srv;

  add_model_srv.request.model_name = model_name;

  while(!add_model_client.call(add_model_srv)) {}

  //Spin
  //The group will exist until the node is shutdown
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}









