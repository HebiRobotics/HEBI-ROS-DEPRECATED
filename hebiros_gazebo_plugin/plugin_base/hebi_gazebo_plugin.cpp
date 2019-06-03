#include "hebi_gazebo_plugin.h"
#include <gazebo/physics/physics.hh>

// Checks the content of a string before the first "." at compile time.
// Necessary because Gazebo only defines string version numbers.
// Note: This requires -std=c++14 or higher to compile
constexpr int GetGazeboVersion (char const* string_ver) {
  int res = 0;
  int i = 0;
  while (string_ver[i] != '\0' && string_ver[i] >= '0' && string_ver[i] <= '9') {
    res *= 10;
    res += static_cast<int>(string_ver[i] - '0');
    ++i;
  }
  return res;
}

// This is a templated struct that allows for wrapping some of the Gazebo
// code for which compilation differse between versions; we use partial
// template specialization to compile the appropriate version.
template<int GazeboVersion, typename JointType> struct GazeboHelper {
  static_assert(GazeboVersion == 7 || GazeboVersion == 9, "Unknown version of gazebo");
  // Default implementations so that the above assertion is the only compilation error
  static double position(JointType joint) { return 0; }
  static double effort(JointType joint) { return 0; }
};

using GazeboWrapper = GazeboHelper<GetGazeboVersion(GAZEBO_VERSION), gazebo::physics::JointPtr>;

template<typename JointType> struct GazeboHelper<7, JointType> {
  static double position(JointType joint) {
    return joint->GetAngle(0).Radian(); 
  }

  static double effort(JointType joint) {
    auto trans = joint->GetChild()->GetInitialRelativePose().rot;
    auto wrench = joint->GetForceTorque(0);
    return (-1 * (trans * wrench.body1Torque)).z;
  }
};

template<typename JointType> struct GazeboHelper<9, JointType> {
  static double position(JointType joint) {
    return joint->Position(0);
  }

  static double effort(JointType joint) {
    auto trans = joint->GetChild()->InitialRelativePose().Rot();
    gazebo::physics::JointWrench wrench = joint->GetForceTorque(0);
    return (-1 * (trans * wrench.body1Torque)).Z();
  }
};

namespace hebi {
namespace sim {
namespace plugin {

void HebiGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;

  // Go through each joint in the model, looking for anything matching a
  // <family>/<name>/<type> naming pattern, where <type> is a recognized
  // module type.
  // We add references to the matching joints so we can simulate them.
  for (auto j : model_->GetJoints())
  {
    auto full_name = j->GetName();
    auto first_slash = full_name.find("/");
    // No slash!
    if (first_slash == std::string::npos)
      continue;
    auto second_slash = full_name.find("/", first_slash + 1);
    if (second_slash == std::string::npos)
      continue;
    
    // Get type:
    auto family = full_name.substr(0, first_slash);
    auto name = full_name.substr(first_slash + 1, second_slash - first_slash - 1);
    auto type = full_name.substr(second_slash + 1);

    auto hebi_joint = Joint::tryCreate(family, name, type);

    if (hebi_joint)
    {
      std::cout << "Adding parsed joint " << family << "/" << name << " of type: " << type << "\n";
      joints_.push_back({ std::move(hebi_joint), j });
    }
  }

  // Call derived class' load:
  onLoad(model, sdf);

  // Set up the update callbacks
  this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin (
    boost::bind(&HebiGazeboPlugin::onUpdateBase, this, _1));
}

void HebiGazeboPlugin::onUpdateBase(const gazebo::common::UpdateInfo& info) {

  auto sim_time = info.simTime;

  // Don't allow dt to be zero...
  if (first_time_) {
    prev_time_ = info.simTime;
    first_time_ = false;
    return;
  }
  auto iteration_time = (sim_time - prev_time_).Double();
  prev_time_ = sim_time;

  for (auto& j : joints_) {
    j.gazebo_joint_->SetProvideFeedback(true);
    double velocity_fbk = j.gazebo_joint_->GetVelocity(0);
    double position_fbk = GazeboWrapper::position(j.gazebo_joint_);
    double effort_fbk = GazeboWrapper::effort(j.gazebo_joint_);

    j.hebi_joint_->update(sim_time.Double(), position_fbk, velocity_fbk, effort_fbk);
    j.hebi_joint_->computePwm(iteration_time);

    double force = j.hebi_joint_->generateForce(iteration_time);

    j.gazebo_joint_->SetForce(0, force);
  }

  // Call derived class' update:
  onUpdate(info);
}
  
Joint* HebiGazeboPlugin::getJoint(const std::string& family, const std::string& name)
{
  auto full_name = family + "/" + name;
  for (auto& joint : joints_)
  {
    if (joint.hebi_joint_->getName() == full_name)
    {
      return joint.hebi_joint_.get();
    }
  }
  return nullptr;
}
  
Joint* HebiGazeboPlugin::getJoint(size_t index)
{
  if (index >= joints_.size())
    return nullptr;
  return joints_[index].hebi_joint_.get();
}

} // namespace plugin
} // namespace sim
} // namespace hebi
