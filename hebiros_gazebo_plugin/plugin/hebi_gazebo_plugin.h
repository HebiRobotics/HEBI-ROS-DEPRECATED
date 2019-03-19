#pragma once

#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include "joint.h"

namespace hebi {
namespace sim {

class HebiGazeboPlugin : public gazebo::ModelPlugin {

public:
  HebiGazeboPlugin() = default;
  
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  // This should be called by implementing functions during their "OnUpdate" callback;
  // this handles all the core joint updates.
  void OnUpdateBase(const gazebo::common::UpdateInfo & info);
  
  gazebo::physics::ModelPtr model_;

  // Returns a raw pointer that can be kept as a weak reference; the unique pointer is
  // stored in this Plugin as a "registry" of all modules.
  Joint* addJoint(std::unique_ptr<Joint> joint);

private:
  std::vector<std::unique_ptr<Joint>> joints_;
};
    
} // namespace sim
} // namespace hebi
