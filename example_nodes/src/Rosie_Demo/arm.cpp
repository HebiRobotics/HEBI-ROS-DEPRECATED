#include "arm.hpp"
#include "lookup.hpp"

#include <cassert>

namespace hebi {
  namespace arm {
    std::unique_ptr<Arm> Arm::createArm( 
      std::vector<std::string> family_name,
      std::vector<std::string> module_names,
      const Eigen::VectorXd& home_position, 
      const ArmKinematics& arm_kinematics,
      double start_time,
      int command_lifetime,
      double feedback_frequency) {

      // Invalid input!  Size mismatch
      if (home_position.size() != arm_kinematics.getModel().getDoFCount() || 
          home_position.size() != std::max(family_name.size(), module_names.size())) {
        assert(false);
        return nullptr;
      }

      Lookup lookup;
      auto group = lookup.getGroupFromNames(family_name, module_names);
      if (!group)
        return nullptr;

      group->setCommandLifetimeMs(command_lifetime);
      group->setFeedbackFrequencyHz(feedback_frequency);

      // Try to get feedback -- if we don't get a packet in the first N times,
      // something is wrong
      int num_attempts = 0;
    
      // This whole "plan initial trajectory" is a little hokey...but it's better than nothing  
      GroupFeedback feedback(group->size());
      while (!group->getNextFeedback(feedback)) {
        if (num_attempts++ > 20) {
          return nullptr;
        }
      } 

      // NOTE: I don't like that start time is _before_ the "get feedback"
      // loop above...but this is only during initialization
      ArmTrajectory arm_trajectory = ArmTrajectory::create(home_position, feedback, start_time);
      return std::unique_ptr<Arm>(new Arm(group, arm_kinematics, arm_trajectory, start_time, home_position));
    }

    // Updates the feedback and sends new commands to the robot for this time
    // step.  Returns 'false' on a connection problem; true on success.
    bool Arm::update(double time) {
      if (!group_->getNextFeedback(feedback_))
        return false;

      // Update command from trajectory
      arm_trajectory_.getState(time, pos_, vel_, accel_);
      command_.setPosition(pos_);
      command_.setVelocity(vel_);

      // Add grav-comp efforts
      command_.setEffort(arm_kinematics_.gravCompEfforts(feedback_));
      // TODO: add dynamic-comp efforts

      // Set color based on current command
      
      for (int i = 0; i < command_.size(); ++i) {
        if (color_.set_color_ == false) 
          command_[i].led().set(hebi::Color(0, 0, 0, 0));
        else
          command_[i].led().set(hebi::Color(
            color_.r_, color_.g_, color_.b_));
      }

      group_->sendCommand(command_);

      return true;
    }


    void Arm::setColor(const Color& c) {
      color_ = c;
    }

    double Arm::trajectoryPercentComplete(double time) {
      return std::min((time - arm_trajectory_.getTrajStartTime()) / arm_trajectory_.getTraj()->getDuration(), 1.0) * 100;
    }

    bool Arm::isTrajectoryComplete(double time) {
      return time > arm_trajectory_.getTrajEndTime();
    }

    Eigen::Vector3d Arm::getHomePositionXYZ() {
      return arm_kinematics_.FK(home_position_);
    }

    Arm::Arm(std::shared_ptr<hebi::Group> group,
      const ArmKinematics& arm_kinematics,
      ArmTrajectory arm_trajectory,
      double start_time,
      const Eigen::VectorXd& home_position)
      : group_{group},
        feedback_(group->size()),
        command_(group->size()),
        home_position_{home_position},
        pos_(Eigen::VectorXd::Zero(group->size())),
        vel_(Eigen::VectorXd::Zero(group->size())),
        accel_(Eigen::VectorXd::Zero(group->size())),
        arm_kinematics_{arm_kinematics},
        arm_trajectory_{arm_trajectory}
    { }

  } // namespace arm_node
} // namespace hebi
