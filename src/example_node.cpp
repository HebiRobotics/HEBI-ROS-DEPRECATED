#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hebi/add_group_from_names_srv.h"

sensor_msgs::JointState feedback;

void feedback_callback(sensor_msgs::JointState data) {
  feedback = data;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "example_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "group0";

  ros::ServiceClient add_group_client = n.serviceClient<hebi::add_group_from_names_srv>(
    "/hebicore/add_group_from_names");
  ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>(
    "/hebicore/"+group_name+"/command/joint_state", 100);
  ros::Subscriber feedback_subscriber = n.subscribe(
    "/hebicore/"+group_name+"/feedback/joint_state", 100, feedback_callback);

  hebi::add_group_from_names_srv add_group_srv;
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"base", "shoulder", "elbow"};
  add_group_srv.request.families = {"HEBI"};

  add_group_client.call(add_group_srv);

  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("/HEBI/base");
  command_msg.name.push_back("/HEBI/shoulder");
  command_msg.name.push_back("/HEBI/elbow");
  command_msg.effort.resize(3);
  feedback.position.reserve(3);

  double spring_constant = -10;

  while(ros::ok()) {

    command_msg.effort[0] = spring_constant * feedback.position[0];
    command_msg.effort[1] = spring_constant * feedback.position[1];
    command_msg.effort[2] = spring_constant * feedback.position[2];
    command_publisher.publish(command_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}









