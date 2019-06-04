#pragma once

#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include "joint.h"

namespace hebi {
namespace sim {
namespace plugin {

// To derive from this base plugin, you must implement the "onLoad" and "onUpdate" functions
class HebiGazeboPlugin : public gazebo::ModelPlugin {

public:
  HebiGazeboPlugin() = default;
  virtual ~HebiGazeboPlugin() = default;

  // NOTE: this does not match code style, because this is overriding a gazebo::ModelPlugin function!
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override final;

protected:
  // This should be implemented by any derived class
  virtual void onLoad(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) = 0;

  // This should be implemented by any derived class
  virtual void onUpdate(const gazebo::common::UpdateInfo& info) = 0;

  // If this joint is found, returns a raw pointer that can be kept as a weak reference.
  // Otherwise, returns nullptr.
  Joint* getJoint(const std::string& family, const std::string& name);

  // Useful for iterating through the joints.  If this exceeds the number of available
  // joints, return `nullptr`.
  Joint* getJoint(size_t index);

  size_t getNumJoints() const { return joints_.size(); }

private:

  // Subscribes to update callbacks from gazebo
  gazebo::event::ConnectionPtr update_connection_;

  // Internal update function -- this is what actually subscribes to gazebo and
  // calls the derived class' onUpdate() function.
  void onUpdateBase(const gazebo::common::UpdateInfo& info);

  struct JointAndModel {
    std::unique_ptr<Joint> hebi_joint_;
    gazebo::physics::JointPtr gazebo_joint_;
  };
  std::vector<JointAndModel> joints_;

  gazebo::physics::ModelPtr model_;

  // The previous time through the loop
  gazebo::common::Time prev_time_;
  bool first_time_;
};
    
} // namespace plugin 
} // namespace sim
} // namespace hebi
