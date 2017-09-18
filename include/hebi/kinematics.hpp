#pragma once

#include "hebi.h"
#include "Eigen/Eigen"
#include "util.hpp"
#include <vector>
#include <memory>

using namespace Eigen;

namespace hebi {
namespace kinematics {

typedef std::vector<Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Matrix4fVector;
typedef std::vector<MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> > MatrixXfVector;
// The result of an IK operation.  More fields will be added to this structure
// in future API releases.
struct IKResult
{
  HebiStatusCode result; // Success or failure
};

class Kinematics;

class Objective
{
friend Kinematics;
protected:
  virtual HebiStatusCode addObjective(HebiIKPtr ik) const = 0;
};

class EndEffectorPositionObjective final : public Objective
{
public:
  EndEffectorPositionObjective(const Eigen::Vector3f&);
  EndEffectorPositionObjective(float weight, const Eigen::Vector3f&);
private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  float _weight, _x, _y, _z;
};

class EndEffectorSO3Objective final : public Objective
{
public:
  EndEffectorSO3Objective(const Eigen::Matrix3f&);
  EndEffectorSO3Objective(float weight, const Eigen::Matrix3f&);
private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  float _weight;
  const float _matrix[9];
};

class EndEffectorTipAxisObjective final : public Objective
{
public:
  EndEffectorTipAxisObjective(const Eigen::Vector3f&);
  EndEffectorTipAxisObjective(float weight, const Eigen::Vector3f&);
private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  float _weight, _x, _y, _z;
};

class JointLimitConstraint final : public Objective
{
public:
  JointLimitConstraint(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions);
  JointLimitConstraint(float weight, const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions);
private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  float _weight;
  Eigen::VectorXd _min_positions;
  Eigen::VectorXd _max_positions;
public:
  // Allow Eigen member variables:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class KinematicBody
{
  friend Kinematics;

  private:
    /**
     * C-style kinematic body object
     */
    HebiBodyPtr internal_;

    /**
     * 'true' if this object is responsible for cleaning up its internal C
     * object, false otherwise.
     */
    bool manage_pointer_lifetime_ = true;

    /**
     * Returns the wrapped C kinematic body object.
     */
    HebiBodyPtr getInternal() const { return internal_; }

    /**
     * Used when/if a Kinematics object consumes this kinematic body. Resources
     * are now controlled by the Kinematics object.
     */
    void consume() { manage_pointer_lifetime_ = false; }

  protected:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * Base constructor called from factory methods to create instances of the
     * KinematicBody class; wraps a C kinematic body object.
     */
    KinematicBody(HebiBodyPtr internal) : internal_(internal) {};
    #endif // DOXYGEN_OMIT_INTERNAL

  public:
    /**
     * \brief Destructor cleans up body (and any child pointers that are still
     * owned).
     */
    virtual ~KinematicBody() noexcept
    {
      if (manage_pointer_lifetime_)
        hebiBodyRelease(internal_);
    }
   
    /**
     * \brief Creates a body with the kinematics of an X5 actuator.
     */ 
    static std::unique_ptr<KinematicBody> createX5();

    /**
     * \brief Creates a body with the kinematics of a tube link between two X5
     * actuators.
     *
     * \param length The center-to-center distance between the actuator
     * rotational axes.
     * \param twist The rotation (in radians) between the actuator axes of
     * rotation. Note that a 0 radian rotation will result in a z-axis offset
     * between the two actuators, and a pi radian rotation will result in the
     * actuator interfaces to this tube being in the same plane, but the
     * rotational axes being anti-parallel.
     */ 
    static std::unique_ptr<KinematicBody> createX5Link(float length, float twist);

    /**
     * \brief Creates a body with the kinematics of a light bracket between two
     * X series actuators.
     *
     * \param mounting This can be set to "Left" or "Right"; invalid options
     * will result in an empty returned unique pointer.
     */ 
    static std::unique_ptr<KinematicBody> createX5LightBracket(HebiMountingType mounting);

    /**
     * \brief Creates a body with the kinematics of a heavy bracket between two
     * X series actuators.
     *
     * \param mounting This can be set to "LeftInside", "LeftOutside",
     * "RightInside", or "RightOutside"; invalid options will result in an empty
     * returned unique pointer.
     */ 
    static std::unique_ptr<KinematicBody> createX5HeavyBracket(HebiMountingType mounting);

    /**
     * \brief Create a generic kinematic body that acts as a fixed transform
     * between an input and output.
     *
     * \param com 3x1 vector of the center of mass location, relative to the input
     * of the kinematic body.
     * \param output 4x4 matrix of the homogeneous transform to the output frame,
     * relative to the input frame of the kinematic body.
     */ 
    static std::unique_ptr<KinematicBody> createGenericLink(const Eigen::Vector3f& com, const Eigen::Matrix4f& output);
};

/**
 * \brief Represents a kinematic chain or tree of bodies (links and joints and
 * modules).
 *
 * (Currently, only kinematic chains are fully supported).
 */
class Kinematics final
{
  friend Objective;

  private:
    /**
     * C-style kinematics object
     */
    const HebiKinematicsPtr internal_;

    /**
     * Internal function for resolving variadic template (stops recursion).
     */
    HebiStatusCode addObjectives(HebiIKPtr ik, const Objective& objective) const
    {
      return objective.addObjective(ik);
    }
    /**
     * Internal function for resolving variadic template (stops recursion and
     * detects invalid arguments).
     */
    template<typename T>
    HebiStatusCode addObjectives(HebiIKPtr ik, const T& not_an_objective) const
    {
      static_assert(std::is_convertible<T*, Objective*>::value,
                    "Must pass arguments of base type hebi::kinematics::Objective to the variable args of solveIK!");
      return HebiStatusInvalidArgument;
    }
    /**
     * Internal function for resolving variadic template.
     */
    template<typename ... Args>
    HebiStatusCode addObjectives(HebiIKPtr ik, const Objective& objective, Args ... args) const
    {
      auto res = objective.addObjective(ik);
      if (res != HebiStatusSuccess)
        return res;
      return addObjectives(ik, args ...);
    }

  public:
    /**
     * \brief Creates a kinematics object with no bodies and an identity base
     * frame.
     */
    Kinematics();

    /**
     * \brief Destructor cleans up kinematics object, including all managed
     * bodies.
     */
    virtual ~Kinematics() noexcept;

    /**
     * \brief Set the transform from a world coordinate system to the input
     * of the root kinematic body in this kinematics object. Defaults to
     * an identity 4x4 matrix.
     *
     * The world coordinate system is used for all position, vector, and
     * transformation matrix parameters in the member functions.
     *
     * \param base_frame The 4x4 homogeneous transform from the world frame to
     * the root kinematic body frame.
     */
    void setBaseFrame(const Eigen::Matrix4f& base_frame);

    /**
     * \brief Returns the transform from the world coordinate system to the root
     * kinematic body, as set by the setBaseFrame function.
     */
    Eigen::Matrix4f getBaseFrame() const;

    /**
     * \brief Return the number of frames in the forward kinematics.
     *
     * Note that this depends on the type of frame requested -- for center of mass
     * frames, there is one per added body; for output frames, there is one per
     * output per body.
     *
     * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
     */
    size_t getFrameCount(HebiFrameType frame_type) const;

    /**
     * \brief Returns the number of settable degrees of freedom in the kinematic
     * tree. (This is equal to the number of actuators added).
     */
    size_t getDoFCount() const;

    /**
     * \brief Add a body to a parent body connected to a kinematic tree object.
     *
     * After the addition, the kinematics object manages the resources of the added
     * body.
     *
     * \param new_body The kinematic body which is added to the tree.
     *
     * \returns true if successful, false if unable to add the body.
     */
    bool addBody(std::unique_ptr<KinematicBody> new_body);

    /**
     * \brief Generates the forward kinematics for the given kinematic tree.
     *
     * See getFK for details.
     */
    void getForwardKinematics(HebiFrameType, const Eigen::VectorXd& positions, Matrix4fVector& frames) const;
    /**
     * \brief Generates the forward kinematics for the given kinematic tree.
     *
     * The order of the returned frames is in a depth-first tree. As an example,
     * assume a body A has one output, to which body B is connected to. Body B has
     * two outputs; actuator C is attached to the first output and actuator E is
     * attached to the second output.  Body D is attached to the only output of
     * actuator C:
     *
     * (BASE) A - B(1) - C - D
     *           (2)
     *            |
     *            E
     *
     * For center of mass frames, the returned frames would be A-B-C-D-E.
     *
     * For output frames, the returned frames would be A-B(1)-C-D-B(2)-E.
     *
     * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
     * \param positions A vector of joint positions/angles (in SI units of meters or
     * radians) equal in length to the number of DoFs of the kinematic tree.
     * \param frames An array of 4x4 transforms; this is resized as necessary
     * in the function and filled in with the 4x4 homogeneous transform of each
     * frame. Note that the number of frames depends on the frame type.
     */
    void getFK(HebiFrameType, const Eigen::VectorXd& positions, Matrix4fVector& frames) const;

    /**
     * \brief Generates the forward kinematics to the end effector (leaf node)
     * frame(s).
     *
     * Note -- for center of mass frames, this is one per leaf node; for output
     * frames, this is one per output per leaf node, in depth first order.
     *
     * This overload is for kinematic chains that only have a single leaf node
     * frame.
     *
     * (Currently, kinematic trees are not fully supported -- only kinematic
     * chains -- and so there are not other overloads for this function).
     *
     * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
     * \param positions A vector of joint positions/angles (in SI units of meters or
     * radians) equal in length to the number of DoFs of the kinematic tree.
     * \param transform A 4x4 transform that is resized as necessary in the
     * function and filled in with the homogeneous transform to the end
     * effector frame.
     */
    void getEndEffector(HebiFrameType, const Eigen::VectorXd& positions, Eigen::Matrix4f& transform) const;

    /**
     * \brief Solves for an inverse kinematics solution given a set of
     * objectives.
     *
     * See solveIK for details.
     */
    template<typename ... Args>
    IKResult solveInverseKinematics(const Eigen::VectorXd& initial_positions, Eigen::VectorXd& result, Args ... args) const
    {
      return solveIK(initial_positions, result, args ...);
    }

    /**
     * \brief Solves for an inverse kinematics solution given a set of
     * objectives.
     *
     * \param initial_positions The seed positions/angles (in SI units of meters
     * or radians) to start the IK search from; equal in length to the number of
     * DoFs of the kinematic tree.
     * \param result A vector equal in length to the number of DoFs of the
     * kinematic tree; this will be filled in with the IK solution (in SI units
     * of meters or radians), and resized as necessary.
     * \param objectives A variable number of objectives used to define the IK
     * search (e.g., target end effector positions, etc). Each argument must
     * have a base class of Objective.
     */
    template<typename ... Args>
    IKResult solveIK(const Eigen::VectorXd& initial_positions,
                     Eigen::VectorXd& result,
                     Args ... objectives) const
    {
      // Create a HEBI C library IK object
      auto ik = hebiIKCreate();

      // (Try) to add objectives to the IK object
      IKResult res;
      res.result = addObjectives(ik, objectives ...);
      if (res.result != HebiStatusSuccess)
      {
        hebiIKRelease(ik);
        return res;
      }

      // Transfer/initialize from Eigen to C arrays
      auto positions_array = new double[initial_positions.size()];
      {
        Map<Eigen::VectorXd> tmp(positions_array, initial_positions.size());
        tmp = initial_positions;
      }
      auto result_array = new double[initial_positions.size()];

      // Call into C library to solve
      res.result = hebiIKSolve(ik, internal_, positions_array, result_array, nullptr);

      // Transfer/cleanup from C arrays to Eigen
      delete[] positions_array;
      {
        Map<Eigen::VectorXd> tmp(result_array, initial_positions.size());
        result = tmp;
      }
      delete[] result_array;

      hebiIKRelease(ik);

      return res;
    }

    /**
     * \brief Generates the Jacobian for each frame in the given kinematic tree.
     *
     * See getJ for details.
     */
    void getJacobians(HebiFrameType, const Eigen::VectorXd& positions, MatrixXfVector& jacobians) const;
    /**
     * \brief Generates the Jacobian for each frame in the given kinematic tree.
     *
     * \param frame_type Which type of frame to consider -- see HebiFrameType
     * enum.
     * \param positions A vector of joint positions/angles (in SI units of
     * meters or radians) equal in length to the number of DoFs of the kinematic
     * tree.
     * \param jacobians A vector (length equal to the number of frames) of
     * matrices; each matrix is a (6 x number of dofs) jacobian matrix for the
     * corresponding frame of reference on the robot.  This vector is resized as
     * necessary inside this function.
     */
    void getJ(HebiFrameType, const Eigen::VectorXd& positions, MatrixXfVector& jacobians) const;

    /**
     * \brief Generates the Jacobian for the end effector (leaf node) frames(s).
     *
     * See getJEndEffector for details.
     */
    void getJacobianEndEffector(HebiFrameType, const Eigen::VectorXd& positions, Eigen::MatrixXf& jacobian) const;
    /**
     * \brief Generates the Jacobian for the end effector (leaf node) frames(s).
     *
     * Note -- for center of mass frames, this is one per leaf node; for output
     * frames, this is one per output per leaf node, in depth first order.
     *
     * This overload is for kinematic chains that only have a single leaf node
     * frame.
     *
     * (Currently, kinematic trees are not fully supported -- only kinematic
     * chains -- and so there are not other overloads for this function).
     *
     * \param frame_type Which type of frame to consider -- see HebiFrameType
     * enum.
     * \param positions A vector of joint positions/angles (in SI units of
     * meters or radians) equal in length to the number of DoFs of the kinematic
     * tree.
     * \param jacobian A (6 x number of dofs) jacobian matrix for the
     * corresponding end effector frame of reference on the robot.  This vector
     * is resized as necessary inside this function.
     */
    void getJEndEffector(HebiFrameType, const Eigen::VectorXd& positions, Eigen::MatrixXf& jacobian) const;

  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(Kinematics)
};

} // namespace kinematics
} // namespace hebi
