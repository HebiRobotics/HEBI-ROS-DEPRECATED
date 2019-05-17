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
    double iteration_time);
  
};
