#include "hebi_gazebo_plugin.h"

namespace hebi {
namespace sim {

void HebiGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;
}

void HebiGazeboPlugin::OnUpdateBase(const gazebo::common::UpdateInfo& info) {
  // TODO: add joint update logic here; but we need to save commands/feedback more usefully via the group objects.
}

Joint* HebiGazeboPlugin::addJoint(std::unique_ptr<Joint> joint) {
  // TODO: keep this from creating duplicate joints for identical model objects...
  // but then we need to refactor groups/joints a bit so the feedback index isn't kept
  // in the group itself.
  auto raw_ptr = joint.get();
  joints_.push_back(std::move(joint));
  return raw_ptr;
}

} // namespace sim
} // namespace hebi
