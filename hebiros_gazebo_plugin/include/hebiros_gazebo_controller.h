
#ifndef _HEBIROS_GAZEBO_CONTROLLER_HH_
#define _HEBIROS_GAZEBO_CONTROLLER_HH_

#include "ros/ros.h"
#include "hebiros_gazebo_group.h"
#include "hebiros_gazebo_joint.h"

using namespace hebiros;


class HebirosGazeboController {

  public:

    enum class control_strategies {
      CONTROL_STRATEGY_OFF = 0,
      CONTROL_STRATEGY_DIRECT_PWM = 1,
      CONTROL_STRATEGY_2 = 2,
      CONTROL_STRATEGY_3 = 3,
      CONTROL_STRATEGY_4 = 4
    };

    static control_strategies DEFAULT_CONTROL_STRATEGY;

    static double MAX_PWM;
    static double MIN_PWM;

    static double LOW_PASS_ALPHA;

    static double DEFAULT_POSITION_KP;
    static double DEFAULT_POSITION_KI;
    static double DEFAULT_POSITION_KD;
    static double DEFAULT_VELOCITY_KP;
    static double DEFAULT_VELOCITY_KI;
    static double DEFAULT_VELOCITY_KD;
    static double DEFAULT_EFFORT_KP;
    static double DEFAULT_EFFORT_KI;
    static double DEFAULT_EFFORT_KD;

    static double GEAR_RATIO_X5_1;
    static double GEAR_RATIO_X5_4;
    static double GEAR_RATIO_X5_9;

    static double DEFAULT_GEAR_RATIO;

    static std::map<std::string, double> gear_ratios;

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
