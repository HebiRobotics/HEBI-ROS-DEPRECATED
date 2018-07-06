/**
* Provides a base level implentation of user controls for a 3-wheeled 
* omnidirectional base (omnibase).
* You can move forward/backward, left/right, or rotate in spot.
* 
* @author Hardik Singh < hardik @ hebirobotics.com >
* @since 6 Jul 2018
**/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/JointState.h"
#include "hebiros/AddGroupFromNamesSrv.h"

#include <ros/console.h>

using namespace hebiros;

// Global Variables
sensor_msgs::JointState feedback;
geometry_msgs::Twist directions;
std::array<double, 3> omniVels; 
std::array<double, 3> omniPos;
double rate_of_command = 60;
bool feedback_init = false;
bool keys_init = false;

void feedback_callback(sensor_msgs::JointState data) {
  /* Callback function which keeps up to date with the feedback from modules*/
  feedback = data;
  feedback_init = true; 
}


void directions_callback(geometry_msgs::Twist data) {
  /* Callback function that keeps up to date with key presses and commands */
  directions = data;
  keys_init = true;
}


void updateOmniVels() {
  /* Declare main kinematic variables */
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (center of omni to origin of base)
  double speed = 0.3; // m/s
  double wheelRotSpeed = M_PI/3; // (rad/s)
  double ratio = sqrt(3)/2;

  // Wheel 1, front right
  omniVels[0] = directions.linear.x * speed * 0.5/wheelRadius + 
                directions.linear.y * speed * -ratio/wheelRadius +
                directions.angular.z * wheelRotSpeed * baseRadius/wheelRadius;

  // Wheel 2, front left
  omniVels[1] = directions.linear.x * speed * 0.5/wheelRadius + 
                directions.linear.y * speed * ratio/wheelRadius +
                directions.angular.z *wheelRotSpeed *  baseRadius/wheelRadius;

  // Wheel 3, back center
  omniVels[2] = directions.linear.x * -speed * 1/wheelRadius + 
                0 +
                directions.angular.z * wheelRotSpeed * baseRadius/wheelRadius;
}


void updatePoseCmd() { 
  /* We assume that we have access to: */
  // - feedback from modules 
  // - omniVels; the velocities being commanded to each module 
  // - rate-of-command; in hertz

  omniPos[0] = omniPos[0] + omniVels[0] * (1/rate_of_command);
  omniPos[1] = omniPos[1] + omniVels[1] * (1/rate_of_command);
  omniPos[2] = omniPos[2] + omniVels[2] * (1/rate_of_command);
}


int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "example_omnibase_free_roam");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  ros::Subscriber key_subscriber = node.subscribe("keys/cmd_vel", 20,
                           directions_callback);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  std::string group_name = "omniGroup";

  // Create a group 
  ros::ServiceClient add_group_client = node.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  // Create a subscriber to receive feedback from a group
  // Register a callback that keeps the feedback updated
  ros::Subscriber feedback_subscriber = node.subscribe(
    "/hebiros/"+group_name+"/feedback/joint_state", 100, feedback_callback);

  // Publisher to send desired commands to a the group
  ros::Publisher command_publisher = node.advertise<sensor_msgs::JointState>(
    "/hebiros/"+group_name+"/command/joint_state", 100);

  // Construct the group using 3 known modules, connected to the network
  AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"_Wheel1", "_Wheel2", "_Wheel3"};
  add_group_srv.request.families = {"Rosie"};

  // Call the add_group_from_urdf service to create a group until it succeeds
  while(!add_group_client.call(add_group_srv)) {}

  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("Rosie/_Wheel1");
  command_msg.name.push_back("Rosie/_Wheel2");
  command_msg.name.push_back("Rosie/_Wheel3");

  command_msg.velocity.resize(3);
  command_msg.position.resize(3);
  feedback.position.resize(3);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                           ////////
  ////////////////////////////////////////////////////////////////////////////

  /******** MAIN LOOP **********/

  bool startup_complete = false;

    while (ros::ok()) {
      if ( !startup_complete ) {
        /* Get the initial position of the wheels at startup */
        if (feedback_init == true &&  keys_init == true) {
          omniPos[0] = feedback.position[0];
          omniPos[1] = feedback.position[1];
          omniPos[2] = feedback.position[2];
          startup_complete = true;
        }

      } else {

      // Get the new velocities and position commands
      updateOmniVels();
      updatePoseCmd();
      
      /* Uncomment to see what velocity commands are being sent */
      //ROS_INFO("%lg %lg %lg", omniVels[0], omniVels[1], omniVels[2]);

      // Set the velocity commands
      command_msg.velocity[0] = omniVels[0];
      command_msg.velocity[1] = omniVels[1];
      command_msg.velocity[2] = omniVels[2];

      /* Uncomment to see what position commands are being sent */
      //ROS_INFO("%lg %lg %lg", omniPos[0], omniPos[1], omniPos[2]);
       
      // Set the position commands
      command_msg.position[0] = omniPos[0];
      command_msg.position[1] = omniPos[1];
      command_msg.position[2] = omniPos[2];

      // Send the commands 
      command_publisher.publish(command_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

