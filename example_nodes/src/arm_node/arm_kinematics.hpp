#include "robot_model.hpp"
#include "group_feedback.hpp"

namespace hebi {
  namespace arm {

    class ArmKinematics {
    public:
      ArmKinematics(const hebi::robot_model::RobotModel& model);

      // Return the joint angles to move to a given xyz location
      Eigen::VectorXd solveIK(
        const Eigen::VectorXd& initial_positions,
        const Eigen::Vector3d& target_xyz) const;

      Eigen::Vector3d FK(const Eigen::VectorXd& positions) const;

      Eigen::VectorXd gravCompEfforts(const hebi::GroupFeedback& feedback) const;

      const hebi::robot_model::RobotModel& getModel() const { return model_; }

    private:
      const hebi::robot_model::RobotModel& model_;
      Eigen::VectorXd masses_;

    };
  } // namespace arm_node
} // namespace hebi
