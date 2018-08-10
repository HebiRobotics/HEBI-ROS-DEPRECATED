#include "arm_kinematics.hpp"
#include "grav_comp.hpp"

namespace hebi {
  namespace arm {

    ArmKinematics::ArmKinematics(const hebi::robot_model::RobotModel& model)
      : model_(model), masses_(model.getFrameCount(HebiFrameTypeCenterOfMass))
    {
      // Update the masses
      model_.getMasses(masses_);
    }

    // Return the joint angles to move to a given xyz location
    Eigen::VectorXd ArmKinematics::solveIK(
      const Eigen::VectorXd& initial_positions,
      const Eigen::Vector3d& target_xyz, 
      const Eigen::Vector3d& end_tip) const
    {
      // NOTE: may want to customize the IK here!

      /*
      // Add joint limit objective?
      //  Eigen::VectorXd min_positions(group -> size());
      //  min_positions << -M_PI_2, -(M_PI*7)/8, -(M_PI*7)/8, -M_PI*2/3;
      //  Eigen::VectorXd max_positions(group -> size());
      //  max_positions << M_PI_2, 0, (M_PI*7)/8, M_PI*2/3;
      */
      /* These joint constraints are specific for the Rosie 6-DoF */
      Eigen::VectorXd min_positions(6);
      min_positions << -M_PI, 0.01, 0.01, -M_PI_2, -M_PI, -M_PI;
      Eigen::VectorXd max_positions(6);
      max_positions << M_PI, M_PI, (M_PI*6)/8, M_PI_2, M_PI, M_PI;

      // TODO: smartly handle exceptions?
      Eigen::VectorXd ik_result_joint_angles(initial_positions.size());
      model_.solveIK(
        initial_positions,
        ik_result_joint_angles,
        robot_model::EndEffectorPositionObjective(target_xyz),
        robot_model::EndEffectorTipAxisObjective(end_tip),
        robot_model::JointLimitConstraint(min_positions, max_positions)
      );
      return ik_result_joint_angles;
    }

    Eigen::Vector3d ArmKinematics::FK(const Eigen::VectorXd& positions) const
    {
      Eigen::Matrix4d transform;
      model_.getEndEffector(positions, transform);
      return Eigen::Vector3d(transform(0,3), transform(1,3), transform(2,3));
    }

    Eigen::VectorXd ArmKinematics::gravCompEfforts(const hebi::GroupFeedback& feedback) const
    {
      return hebi::util::GravityCompensation::getEfforts(model_, masses_, feedback);
    }
    
  } // namespace arm_node
} // namespace hebi
