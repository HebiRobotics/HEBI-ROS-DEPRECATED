#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

#include <sstream>
#include <string>
#include <vector>

/**
 * A short function to get name and family from a name split with a "|".
 */
bool split(const std::string &orig, std::string &name, std::string &family)
{
  std::stringstream ss(orig);
  if (!std::getline(ss, name, '|'))
    return false;
  std::getline(ss, family);
  return true;
}

// Global 'group' pointer so we can access this in a callback...ugly, but until
// we class-ify the node this will work.
hebi::Group* group_g = NULL;

/**
 * Respond to joint commands.
 */
void commandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard from: [%s] with [%f]", msg->name[0].c_str(), msg->position[0]);
  // TODO: check names instead of just length?
  if ((group_g->size() != msg->position.size() && msg->position.size() != 0) ||
      (group_g->size() != msg->velocity.size() && msg->velocity.size() != 0) ||
      (group_g->size() != msg->effort.size() && msg->effort.size() != 0))
  {
    ROS_INFO("Command length did not match group size.");
    return;
  }
  hebi::GroupCommand cmd(group_g->size());
  for (int i = 0; i < msg->position.size(); i++)
    cmd[i].actuator().position().set(msg->position[i]);
  for (int i = 0; i < msg->velocity.size(); i++)
    cmd[i].actuator().velocity().set(msg->velocity[i]);
  for (int i = 0; i < msg->effort.size(); i++)
    cmd[i].actuator().torque().set(msg->effort[i]);
  group_g->sendCommand(cmd);
}

void add_joint_children(std::vector<std::string>& names, std::vector<std::string>& families, std::vector<std::string>& full_names, const urdf::Link* link)
{
  for (auto& joint : link->child_joints)
  {
    // Actually add this if it is a joint.
    if (joint->type != urdf::Joint::FIXED)
    {
      std::string name, family;
      if (split(joint->name, name, family))
      {
        names.push_back(name);
        families.push_back(family);
        full_names.push_back(joint->name);
      }
    }
  }
  // Recurse
  for (auto& link_child : link->child_links)
    add_joint_children(names, families, full_names, link_child.get());
}

/**
 * This node publishes feedback from named joints in the URDF model on the
 * parameter server under "robot_description".
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "group");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  std::string urdf_name("robot_description");

  if (!n.hasParam(urdf_name))
  {
    ROS_INFO("Could not find robot description parameter! Quitting!");
    return -1;
  }

  urdf::Model urdf_model;
  if (!urdf_model.initParam(urdf_name))
  {
    ROS_INFO("Could not load robot description! Quitting!");
    return -1;
  }

  // Get the names of non-fixed joints in the model:
  std::vector<std::string> joint_names;
  std::vector<std::string> family_names;
  std::vector<std::string> joint_full_names;
  add_joint_children(joint_names, family_names, joint_full_names, urdf_model.getRoot().get());

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  tf::TransformBroadcaster broadcaster;

  // Get the lookup and wait to populate
  hebi::Lookup lookup;
  sleep(2);
  lookup.printTable();

  // Get the group
  for (int i = 0; i < joint_names.size(); i++)
  {
    std::cout << "looking for: " << std::endl;
    std::cout << joint_names[i] << std::endl;
    std::cout << family_names[i] << std::endl;
  }
  std::unique_ptr<hebi::Group> group(lookup.getGroupFromNames(joint_names, family_names, 1000));
  if (!group)
  {
    ROS_INFO("Could not find modules on network! Quitting!");
    return -1;
  }

  std::cout << "Found modules!" << std::endl;
  ROS_INFO("Found modules!");

  // THIS IS A HACK to get around limited callback options for ROS subscribe call and the lack of a class for this node.
  group_g = group.get();

  // Subscribe to the 'joint commands' channel to respond to desired commands
  // sent to this node.
  ros::Subscriber sub = n.subscribe("joint_commands", 5, commandCallback);

  // declare odometry message, required by robot state publisher.
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base";
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

  // Add an async feedback handler, sending feedback from the modules on joint_states
  group->addFeedbackHandler(
    [&joint_pub, &broadcaster, &odom_trans, &joint_full_names](const hebi::GroupFeedback* fbk)->void
      {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        // Add feedback:
        for (int i = 0; i < joint_full_names.size(); i++)
        {
          msg.name.push_back(joint_full_names[i].c_str());
          if ((*fbk)[i].actuator().position().has())
            msg.position.push_back((*fbk)[i].actuator().position().get());
          if ((*fbk)[i].actuator().velocity().has())
            msg.velocity.push_back((*fbk)[i].actuator().velocity().get());
          if ((*fbk)[i].actuator().torque().has())
            msg.effort.push_back((*fbk)[i].actuator().torque().get());
        }

        odom_trans.header.stamp = ros::Time::now();

        joint_pub.publish(msg);
        broadcaster.sendTransform(odom_trans);
      });
  // Set the rate at which this gets called.  (TODO: make this a parameter)
  group->setFeedbackFrequencyHz(25);

  while (ros::ok())
  {
    sleep(1);
    ros::spinOnce();
  }

  // Stop the async callback before returning and deleting objects.
  group->clearFeedbackHandlers();

  sleep(1); // wait for cleanup

  return 0;
}
