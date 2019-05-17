#pragma once

#include "ros/ros.h"
#include "joint.h"
#include "hebiros_gazebo_group.h"

using namespace hebiros;

class HebirosGazeboController {

public:

  HebirosGazeboController() = default;
  
  static double ComputeForce(
    hebi::sim::Joint* hebiros_joint,
    double position, double velocity, double effort, double iteration_time);
  
  static void SetSettings(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
    hebi::sim::Joint* hebiros_joint);
  
  static void SetDefaultGains(std::shared_ptr<HebirosGazeboGroup> hebiros_group, 
    hebi::sim::Joint* hebiros_joint);
  
  static double Clip(double x, double low, double high);

};
