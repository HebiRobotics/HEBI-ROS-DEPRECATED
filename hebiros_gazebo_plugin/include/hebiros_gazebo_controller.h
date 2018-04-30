
#ifndef _HEBIROS_GAZEBO_CONTROLLER_HH_
#define _HEBIROS_GAZEBO_CONTROLLER_HH_

#include "ros/ros.h"
#include "hebiros_gazebo_group.h"
#include "hebiros_gazebo_joint.h"

using namespace hebiros;


class HebirosGazeboController {

  public:

    HebirosGazeboController();
    ~HebirosGazeboController();
    static double ComputeForce(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
      double position, double velocity, double effort);
    static double ComputePositionPID(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
      double target_position, double position, ros::Duration iteration_time);
    static double ComputeVelocityPID(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
      double target_velocity, double velocity, ros::Duration iteration_time);
    static double ComputeEffortPID(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
      double target_effort, double effort, ros::Duration iteration_time);
    static void SetSettings(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint);
    static void ChangeSettings(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint);
    static void SetDefaultGains(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
      std::shared_ptr<HebirosGazeboJoint> hebiros_joint);
    static double Clip(double x, double low, double high);

};


#endif
