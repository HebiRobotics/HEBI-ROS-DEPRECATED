#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SendCommandWithAcknowledgementSrv.h"

using namespace hebiros;


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_set_gains_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "my_group";

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  //Create a client which uses the service to send a command and determine
  //whether it has been received
  ros::ServiceClient send_command_client = n.serviceClient<SendCommandWithAcknowledgementSrv>(
    "/hebiros/"+group_name+"/send_command_with_acknowledgement");

  AddGroupFromNamesSrv add_group_srv;

  //Construct a group using 3 known modules
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"base", "shoulder", "elbow"};
  add_group_srv.request.families = {"HEBI"};
  //Call the add_group_from_names service to create a group until it succeeds
  //Specific topics and services will now be available under this group's namespace
  while(!add_group_client.call(add_group_srv)) {}

  //Construct a CommandMsg to command the modules
  //In addition to joint states, various settings such as pid gains may be set
  CommandMsg command_msg;
  std::vector<std::string> names = {"HEBI/base", "HEBI/shoulder", "HEBI/elbow"};

  command_msg.name = names;
  command_msg.settings.name = names;
  //Set each of the modules to not save settings after reboot (default behavior)
  command_msg.settings.save_current_settings = {false, false, false};
  command_msg.settings.position_gains.name = names;
  command_msg.settings.velocity_gains.name = names;
  command_msg.settings.effort_gains.name = names;
  //For each module, set p, i, d gains for position, velocity, effort
  command_msg.settings.position_gains.kp = {1, 2, 1};
  command_msg.settings.position_gains.ki = {0.1, 0.2, 0.2};
  command_msg.settings.position_gains.kd = {0.005, 0.001, 0.001};
  command_msg.settings.velocity_gains.kp = {0.1, 0.2, 0.1};
  command_msg.settings.velocity_gains.ki = {0.01, 0.02, 0.02};
  command_msg.settings.velocity_gains.kd = {0.005, 0.001, 0.001};
  command_msg.settings.effort_gains.kp = {0.001, 0.002, 0.001};
  command_msg.settings.effort_gains.ki = {0.001, 0.002, 0.001};
  command_msg.settings.effort_gains.kd = {0.005, 0.001, 0.001};

  SendCommandWithAcknowledgementSrv send_command_srv;
  send_command_srv.request.command = command_msg;

  //Send the command and check for acknowledgement
  //It is also possible to publish the command over a topic without acknowledgement
  if (send_command_client.call(send_command_srv)) {
    std::cout << "The gains were changed successfully" << std::endl;
  }
  else {
    std::cout << "The gains were not changed" << std::endl;
  }

  return 0;
}









