#pragma once

#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include "joint.h"

namespace hebi {
namespace sim {
namespace plugin {

class HebiGazeboPlugin : public gazebo::ModelPlugin {

public:
  HebiGazeboPlugin() = default;
  virtual ~HebiGazeboPlugin() = default;
  
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  // This should be called by implementing functions during their "OnUpdate" callback;
  // this handles all the core joint updates.
  void OnUpdateBase(const gazebo::common::UpdateInfo & info);
  
  gazebo::physics::ModelPtr model_;

  // If this joint is found, returns a raw pointer that can be kept as a weak reference.
  // Otherwise, returns nullptr.
  Joint* getJoint(const std::string& family, const std::string& name);

  // Useful for iterating through the joints.  If this exceeds the number of available
  // joints, return `nullptr`.
  Joint* getJoint(size_t index);

  size_t getNumJoints() const { return joints_.size(); }

private:
  struct JointAndModel {
    std::unique_ptr<Joint> hebi_joint;
    gazebo::physics::JointPtr gazebo_joint;
  };
  std::vector<JointAndModel> joints_;

  // The previous time through the loop
  gazebo::common::Time prev_time_;
  bool first_time_;
};
    
} // namespace plugin 
} // namespace sim
} // namespace hebi
