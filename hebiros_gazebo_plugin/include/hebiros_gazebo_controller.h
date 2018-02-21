
#ifndef _HEBIROS_GAZEBO_CONTROLLER_HH_
#define _HEBIROS_GAZEBO_CONTROLLER_HH_

#include "ros/ros.h"
#include "hebiros_gazebo_joint.h"
#include <map>

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

    control_strategies DEFAULT_CONTROL_STRATEGY = control_strategies::CONTROL_STRATEGY_3;

    double MAX_PWM = 1.0;
    double MIN_PWM = -1.0;

    double DEFAULT_POSITION_KP = 1.0;
    double DEFAULT_POSITION_KI = 0.0;
    double DEFAULT_POSITION_KD = 0.0;
    double DEFAULT_VELOCITY_KP = 0.05;
    double DEFAULT_VELOCITY_KI = 0.0;
    double DEFAULT_VELOCITY_KD = 0.0;
    double DEFAULT_EFFORT_KP = 0.25;
    double DEFAULT_EFFORT_KI = 0.0;
    double DEFAULT_EFFORT_KD = 0.001;

    double GEAR_RATIO_X5_1 = 272.22;
    double GEAR_RATIO_X5_4 = 762.22;
    double GEAR_RATIO_X5_9 = 1742.22;

    double DEFAULT_GEAR_RATIO = GEAR_RATIO_X5_1;

    std::map<std::string, double> gear_ratios = {
      {"X5-1", GEAR_RATIO_X5_1},
      {"X5-4", GEAR_RATIO_X5_4},
      {"X5-9", GEAR_RATIO_X5_9}};

    HebirosGazeboController();
    ~HebirosGazeboController();
    double ComputeForce(std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
      double position, double velocity, double effort);
    void SetSettings(std::shared_ptr<HebirosGazeboJoint> hebiros_joint);
    double Clip(double x, double low, double high);

};


#endif
