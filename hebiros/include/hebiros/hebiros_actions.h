#ifndef HEBIROS_ACTIONS_H
#define HEBIROS_ACTIONS_H

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "hebiros/TrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

class HebirosActions {

  public:

    static std::map<std::string,
      std::shared_ptr<actionlib::SimpleActionServer<hebiros::TrajectoryAction>>>
      trajectory_actions;

    static std::map<std::string,
      std::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>>
      follow_joint_trajectory_actions;

    void registerGroupActions(std::string group_name);
    void trajectory(const hebiros::TrajectoryGoalConstPtr& goal, std::string group_name);
    void follow_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, std::string group_name);

};

#endif
