#include "kinematics.hpp"
#include "hebi.h"

namespace hebi {
namespace kinematics {

////////////////////////// Objectives

EndEffectorPositionObjective::EndEffectorPositionObjective(const Eigen::Vector3f& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

EndEffectorPositionObjective::EndEffectorPositionObjective(float weight, const Eigen::Vector3f& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

HebiStatusCode EndEffectorPositionObjective::addObjective(HebiIKPtr ik) const
{
  return hebiIKAddObjectiveEndEffectorPosition(ik, _weight, _x, _y, _z);
}

EndEffectorSO3Objective::EndEffectorSO3Objective(const Eigen::Matrix3f& matrix)
  : _weight(1.0f), _matrix{
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2)}
{ }

EndEffectorSO3Objective::EndEffectorSO3Objective(float weight, const Eigen::Matrix3f& matrix)
  : _weight(weight), _matrix{
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2)}
{ }

HebiStatusCode EndEffectorSO3Objective::addObjective(HebiIKPtr ik) const
{
  return hebiIKAddObjectiveEndEffectorSO3(ik, _weight, _matrix);
}

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(const Eigen::Vector3f& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(float weight, const Eigen::Vector3f& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

HebiStatusCode EndEffectorTipAxisObjective::addObjective(HebiIKPtr ik) const
{
  return hebiIKAddObjectiveEndEffectorTipAxis(ik, _weight, _x, _y, _z);
}

JointLimitConstraint::JointLimitConstraint(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions)
  : _weight(1.0f), _min_positions(min_positions), _max_positions(max_positions)
{ } 

JointLimitConstraint::JointLimitConstraint(float weight, const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions)
  : _weight(weight), _min_positions(min_positions), _max_positions(max_positions)
{ } 

HebiStatusCode JointLimitConstraint::addObjective(HebiIKPtr ik) const
{
  if (_min_positions.size() != _max_positions.size())
    return HebiStatusInvalidArgument;

  int num_joints = _min_positions.size();

  auto min_positions_array = new double[num_joints];
  {
    Map<Eigen::VectorXd> tmp(min_positions_array, num_joints);
    tmp = _min_positions;
  }
  auto max_positions_array = new double[num_joints];
  {
    Map<Eigen::VectorXd> tmp(max_positions_array, num_joints);
    tmp = _max_positions;
  }
  
  auto res = hebiIKAddConstraintJointAngles(ik, _weight, num_joints, min_positions_array, max_positions_array);

  delete[] min_positions_array;
  delete[] max_positions_array;

  return res;
}

////////////////////////// Kinematic Bodies

std::unique_ptr<KinematicBody> KinematicBody::createX5()
{
  HebiKinematicParametersActuator params;
  hebiKinematicParametersX5(&params);

  auto tmp = hebiBodyCreateActuator(
    params.com,
    params.input_to_joint,
    params.joint_rotation_axis,
    params.joint_to_output);
  return std::unique_ptr<KinematicBody>(new KinematicBody(tmp));
}

std::unique_ptr<KinematicBody> KinematicBody::createX5Link(float length, float twist)
{
  HebiKinematicParametersStaticBody params;
  hebiKinematicParametersX5Link(&params, length, twist);
  auto tmp = hebiBodyCreateStatic(params.com, 1, params.output);
  return std::unique_ptr<KinematicBody>(new KinematicBody(tmp));
}

std::unique_ptr<KinematicBody> KinematicBody::createX5LightBracket(HebiMountingType mounting)
{
  std::unique_ptr<KinematicBody> new_body;
  HebiKinematicParametersStaticBody params;
  if (hebiKinematicParametersX5LightBracket(&params, mounting) == HebiStatusSuccess)
  {
    auto tmp = hebiBodyCreateStatic(params.com, 1, params.output);
    new_body.reset(new KinematicBody(tmp));
  }
  return new_body;
}

std::unique_ptr<KinematicBody> KinematicBody::createX5HeavyBracket(HebiMountingType mounting)
{
  std::unique_ptr<KinematicBody> new_body;
  HebiKinematicParametersStaticBody params;
  if (hebiKinematicParametersX5HeavyBracket(&params, mounting) == HebiStatusSuccess)
  {
    auto tmp = hebiBodyCreateStatic(params.com, 1, params.output);
    new_body.reset(new KinematicBody(tmp));
  }
  return new_body;
}

std::unique_ptr<KinematicBody> KinematicBody::createGenericLink(const Eigen::Vector3f& com, const Eigen::Matrix4f& output)
{
  float c_com[3];
  float c_output[16];
  Map<Matrix<float, 3, 1> > tmp_com(c_com);
  Map<Matrix<float, 4, 4, RowMajor> > tmp_output(c_output);
  tmp_com = com;
  tmp_output = output;
  
  auto tmp = hebiBodyCreateStatic(c_com, 1, c_output);
  return std::unique_ptr<KinematicBody>(new KinematicBody(tmp));
}

////////////////////////// Kinematics

Kinematics::Kinematics()
  : internal_(hebiKinematicsCreate())
{
}

Kinematics::~Kinematics() noexcept
{
  hebiKinematicsRelease(internal_);
}

void Kinematics::setBaseFrame(const Eigen::Matrix4f& base_frame)
{
  // Put data into an array
  float transform[16];
  Map<Matrix<float, 4, 4, RowMajor> > tmp(transform);
  tmp = base_frame;
  hebiKinematicsSetBaseFrame(internal_, transform);
}

Eigen::Matrix4f Kinematics::getBaseFrame() const
{
  // Get the data into an array
  float transform[16];
  hebiKinematicsGetBaseFrame(internal_, transform);

  // Copy out data
  Map<const Matrix<float, 4, 4, RowMajor> > tmp(transform);
  Eigen::Matrix4f res;
  res = tmp;
  return res;
}

size_t Kinematics::getFrameCount(HebiFrameType frame_type) const
{
  int res = hebiKinematicsGetNumberOfFrames(internal_, frame_type);
  if (res < 0)
    return 0;
  return (size_t)res;
}

size_t Kinematics::getDoFCount() const
{
  int res = hebiKinematicsGetNumberOfDoFs(internal_);
  if (res < 0)
    return 0;
  return (size_t)res;
}

// TODO: handle trees/etc by passing in parent object here, and output index
bool Kinematics::addBody(std::unique_ptr<KinematicBody> body)
{
  bool was_added = (hebiKinematicsAddBody(internal_, nullptr, 0, body->getInternal()) == 0);
  if (was_added)
    body->consume();
  body.reset();
  return was_added;
}

void Kinematics::getForwardKinematics(HebiFrameType frame_type, const Eigen::VectorXd& positions, Matrix4fVector& frames) const
{
  getFK(frame_type, positions, frames);
}
void Kinematics::getFK(HebiFrameType frame_type, const Eigen::VectorXd& positions, Matrix4fVector& frames) const
{
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }
  size_t num_frames = getFrameCount(frame_type);
  auto frame_array = new float[16 * num_frames];
  // Get data from C API
  hebiKinematicsGetForwardKinematics(internal_, frame_type, positions_array, frame_array);
  delete[] positions_array;
  // Copy into vector of matrices passed in
  frames.resize(num_frames);
  for (int i = 0; i < num_frames; ++i)
  {
    Map<Matrix<float, 4, 4, RowMajor> > tmp(frame_array + i * 16);
    frames[i] = tmp;
  }
  delete[] frame_array;
}

void Kinematics::getEndEffector(HebiFrameType frame_type, const Eigen::VectorXd& positions, Eigen::Matrix4f& transform) const
{
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }

  float transform_array[16];
  hebiKinematicsGetEndEffector(internal_, frame_type, positions_array, transform_array);
  delete[] positions_array;
  {
    Map<Matrix<float, 4, 4, RowMajor> > tmp(transform_array);
    transform = tmp;
  }
}

void Kinematics::getJacobians(HebiFrameType frame_type, const Eigen::VectorXd& positions, MatrixXfVector& jacobians) const
{
  getJ(frame_type, positions, jacobians);
}
void Kinematics::getJ(HebiFrameType frame_type, const Eigen::VectorXd& positions, MatrixXfVector& jacobians) const
{
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }

  size_t num_frames = getFrameCount(frame_type);
  size_t num_dofs = positions.size();
  size_t rows = 6 * num_frames;
  size_t cols = num_dofs;
  auto jacobians_array = new float[rows * cols];
  hebiKinematicsGetJacobians(internal_, frame_type, positions_array, jacobians_array);
  delete[] positions_array;
  jacobians.resize(num_frames);
  for (int i = 0; i < num_frames; ++i)
  {
    Map<Matrix<float, Dynamic, Dynamic, RowMajor> > tmp(jacobians_array + i * cols * 6, 6, cols);
    jacobians[i] = tmp;
  }
  delete[] jacobians_array;
}
void Kinematics::getJacobianEndEffector(HebiFrameType frame_type, const Eigen::VectorXd& positions, Eigen::MatrixXf& jacobian) const
{
  getJEndEffector(frame_type, positions, jacobian);
}
void Kinematics::getJEndEffector(HebiFrameType frame_type, const Eigen::VectorXd& positions, Eigen::MatrixXf& jacobian) const
{
  MatrixXfVector tmp_jacobians;
  getJacobians(frame_type, positions, tmp_jacobians);

  // NOTE: could make this more efficient by writing additional lib function
  // for this, instead of tossing away almost everything from the full one!

  size_t num_dofs = positions.size();
  jacobian.resize(6, num_dofs);
  jacobian = *tmp_jacobians.rbegin();
}

} // namespace kinematics
} // namespace hebi
