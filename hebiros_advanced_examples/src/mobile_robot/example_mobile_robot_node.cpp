#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include "sensor_msgs/JointState.h"
#include "hebiros/AddGroupFromNamesSrv.h"

using namespace hebiros;


double x = 0.0;
double y = 0.0;
double th = 0.0;
geometry_msgs::Quaternion odom_quat;

double vx = 0;
double vy = 0;
double vth = 0;

//Radius of the wheel (m)
double wheel_radius = 0.1;
//Distance from the center to the wheel (m)
double wheel_distance = 0.087;

//Set linear and angular velocity commands based on incoming /cmd_vel
void cmd_vel_callback(geometry_msgs::Twist msg){
  vx = msg.linear.x;
  vth = msg.angular.z;
}

//Reset odometry position to 0
bool reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  x = 0.0;
  y = 0.0;
  th = 0.0;

  return true;
}

//Set odometry position and orientation from rtabmap
void odometry_callback(nav_msgs::Odometry data) {
  x = data.pose.pose.position.x;
  y = data.pose.pose.position.y;
  odom_quat = data.pose.pose.orientation;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "example_mobile_robot_node");
  ros::NodeHandle node;

  //This node while handle two transforms, one from "map" to "odom"
  //and one from "odom" to "base_link" (the robot)
  tf::TransformBroadcaster map_broadcaster;
  tf::TransformBroadcaster odom_broadcaster;

  odom_quat.x = 0.0;
  odom_quat.y = 0.0;
  odom_quat.z = 0.0;
  odom_quat.w = 1.0;

  std::string group_name = "mobile_robot";

  //Convenience service for resetting the odometry position to 0
  ros::ServiceServer reset_service = node.advertiseService(
    "/example_mobile_robot_node/reset", &reset_callback);

  //Receive visual odometry from rtabmap
  ros::Subscriber odometry_subscriber = node.subscribe(
    "/rtabmap/odom", 100, odometry_callback);

  //Publish odometry for move_base by copying the received odometry from rtabmap
  //It would also be possible to fuse rtabmap odometry with other odometry sources before publishing
  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);

  //Subscribe to command velocities for the robot to be translated into left and right velocities
  ros::Subscriber cmd_vel_subscriber = node.subscribe("/cmd_vel", 10, &cmd_vel_callback);

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = node.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  //Create a publisher to send desired commands to a group
  ros::Publisher command_publisher = node.advertise<sensor_msgs::JointState>(
    "/hebiros/"+group_name+"/command/joint_state", 100);

  //Construct a group using 2 known modules
  AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"Left", "Right"};
  add_group_srv.request.families = {"Wheels"};
  add_group_client.call(add_group_srv);

  //Construct a JointState to command the modules
  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("Wheels/Left");
  command_msg.name.push_back("Wheels/Right");
  command_msg.velocity.resize(2);

  ros::Rate rate(200.0);

  while(ros::ok()){
    ros::spinOnce();

    ros::Time current_time = ros::Time::now();

    //Send the transform for "map" to "odom" to tf
    //The map and odometry frame do not differ, so just send a 0 transform
    map_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        current_time,"map", "odom"));

    //Send the transform for "odom" to "base_link" to tf
    //The odometry position and orientation are simply copied from rtabmap's odometry
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    //Additionally publish the odometry over a rostopic
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    //The odometry position and orientation are simply copied from rtabmap's odometry
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //The twist is taken from the commanded velocity over /cmd_vel
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);

    //Apply standard differential drive equations to the wheel modules and publish as a command
    //The right wheel velocity is negative because it is mounted in the opposite direction
    command_msg.velocity[0] = 1.0 * ((vx - (wheel_distance * vth)) / wheel_radius);
    command_msg.velocity[1] = -1.0 * ((vx + (wheel_distance * vth)) / wheel_radius);
    command_publisher.publish(command_msg);

    rate.sleep();
  }

  return 0;
}









