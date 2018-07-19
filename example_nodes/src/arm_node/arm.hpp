#include "group_command.hpp"
#include "arm_trajectory.hpp"
#include "arm_kinematics.hpp"

namespace hebi {
  namespace arm {

    // A high-level abstraction of a robot arm, coordinating kinematics, control,
    // and basic motion planning.
    class Arm {
    public:
      static std::unique_ptr<Arm> createArm( 
        std::vector<std::string> family_name,
        std::vector<std::string> module_names,
        const Eigen::VectorXd& home_position, 
        const ArmKinematics& arm_kinematics,
        double start_time,
        int command_lifetime = 100, // ms
        double feedback_frequency = 200.0); // hz

      // Updates the feedback and sends new commands to the robot for this time
      // step.  Returns 'false' on a connection problem; true on success.
      bool update(double time);

      size_t size() { return group_->size(); }
      GroupFeedback& getLastFeedback() { return feedback_; }

      const ArmKinematics& getKinematics() { return arm_kinematics_; }
      ArmTrajectory& getTrajectory() { return arm_trajectory_; }

    private:
      Arm(std::shared_ptr<hebi::Group> group,
        const ArmKinematics& arm_kinematics,
        ArmTrajectory arm_trajectory,
        double start_time);

      std::shared_ptr<hebi::Group> group_;
      hebi::GroupFeedback feedback_;
      hebi::GroupCommand command_;

      // These are just temporary variables to cache output from
      // Trajectory::getState.
      Eigen::VectorXd pos_;
      Eigen::VectorXd vel_;
      Eigen::VectorXd accel_;

      const ArmKinematics& arm_kinematics_;
      ArmTrajectory arm_trajectory_;

    };
  } // namespace arm_node
} // namespace hebi
