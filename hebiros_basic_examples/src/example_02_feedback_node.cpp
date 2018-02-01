#include "ros/ros.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"

using namespace hebiros;


//Global variable and callback function used to store feedback data
FeedbackMsg feedback;

void feedback_callback(FeedbackMsg data) {
  feedback = data;
}


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_02_feedback_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "my_group";

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  //Create a client which uses the service to find the size of a group
  ros::ServiceClient size_client = n.serviceClient<SizeSrv>(
    "/hebiros/"+group_name+"/size");

  //Create a subscriber to receive feedback from a group
  //Register feedback_callback as a callback which runs when feedback is received
  ros::Subscriber feedback_subscriber = n.subscribe(
    "/hebiros/"+group_name+"/feedback", 100, feedback_callback);

  AddGroupFromNamesSrv add_group_srv;
  SizeSrv size_srv;

  //Construct a group using 3 known modules
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"base", "shoulder", "elbow"};
  add_group_srv.request.families = {"HEBI"};
  //Call the add_group_from_urdf service to create a group until it succeeds
  //Specific topics and services will now be available under this group's namespace
  while(!add_group_client.call(add_group_srv)) {}

  //Call the size service for the newly created group
  size_client.call(size_srv);

  feedback.accelerometer.reserve(size_srv.response.size);
  feedback.gyro.reserve(size_srv.response.size);

  while(ros::ok()) {

    //Display IMU data for each module
    //You can also see feedback by running "rostopic echo /hebiros/<group>/feedback"
    for (int i = 0; i < size_srv.response.size; i++) {
      std::cout << "Module " << i << ": " << std::endl;
      
      if (feedback.m_stop_triggered.size() > i && feedback.m_stop_triggered[i])
        std::cout << "MSTOP TRIGGERED" << std::endl;
    }
    std::cout << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}









