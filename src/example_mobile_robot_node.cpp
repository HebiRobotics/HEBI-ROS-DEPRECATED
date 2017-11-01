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

double wheel_radius = 0.2;
double wheel_distance = 0.2;

void cmd_vel_callback(geometry_msgs::Twist msg){
  vx = msg.linear.x;
  vth = msg.angular.z;
}

bool reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  x = 0.0;
  y = 0.0;
  th = 0.0;

  return true;
}

void odometry_callback(nav_msgs::Odometry data) {
  x = data.pose.pose.position.x;
  y = data.pose.pose.position.y;
  odom_quat = data.pose.pose.orientation;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "example_mobile_robot_node");
  ros::NodeHandle node;

  odom_quat.w = 1.0;

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Subscriber cmd_vel_subscriber = node.subscribe("/cmd_vel", 10, &cmd_vel_callback);

  tf::TransformBroadcaster map_broadcaster;
  tf::TransformBroadcaster odom_broadcaster;


  std::string group_name = "mobile_robot";

  ros::ServiceClient add_group_client = node.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"Left", "Right"};
  add_group_srv.request.families = {"Wheels"};
  add_group_client.call(add_group_srv);

  ros::Publisher command_publisher = node.advertise<sensor_msgs::JointState>(
    "/hebiros/"+group_name+"/command/joint_state", 100);

  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("Wheels/Left");
  command_msg.name.push_back("Wheels/Right");
  command_msg.velocity.resize(2);

  ros::ServiceServer reset_service = node.advertiseService(
    "/example_mobile_robot_node/reset", &reset_callback);

  ros::Subscriber odometry_subscriber = node.subscribe(
    "/rtabmap/odom", 100, odometry_callback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate rate(200.0);

  while(ros::ok()){
    ros::spinOnce();

    current_time = ros::Time::now();

    map_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        current_time,"map", "odom"));


    command_msg.velocity[0] = 0.5 * ((vx - (wheel_distance * vth)) / wheel_radius);
    command_msg.velocity[1] = -0.5 * ((vx + (wheel_distance * vth)) / wheel_radius);
    command_publisher.publish(command_msg);


    //compute odometry in a typical way given the velocities of the robot
    //double dt = (current_time - last_time).toSec();
    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    //double delta_th = vth * dt;

    //x += delta_x;
    //y += delta_y;
    //th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    rate.sleep();
  }

  return 0;
}









