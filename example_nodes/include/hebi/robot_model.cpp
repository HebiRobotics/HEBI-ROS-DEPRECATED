#include "robot_model.hpp"
#include "hebi.h"

namespace hebi {
namespace robot_model {

////////////////////////// Objectives

EndEffectorPositionObjective::EndEffectorPositionObjective(const Eigen::Vector3d& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

EndEffectorPositionObjective::EndEffectorPositionObjective(double weight, const Eigen::Vector3d& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

HebiStatusCode EndEffectorPositionObjective::addObjective(HebiIKPtr ik) const
{
  return hebiIKAddObjectiveEndEffectorPosition(ik, static_cast<float>(_weight), 0, _x, _y, _z);
}

EndEffectorSO3Objective::EndEffectorSO3Objective(const Eigen::Matrix3d& matrix)
  : _weight(1.0f), _matrix{
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2)}
{ }

EndEffectorSO3Objective::EndEffectorSO3Objective(double weight, const Eigen::Matrix3d& matrix)
  : _weight(weight), _matrix{
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2)}
{ }

HebiStatusCode EndEffectorSO3Objective::addObjective(HebiIKPtr ik) const
{
  return hebiIKAddObjectiveEndEffectorSO3(ik, _weight, 0, _matrix);
}

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(const Eigen::Vector3d& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(double weight, const Eigen::Vector3d& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2])
{ } 

HebiStatusCode EndEffectorTipAxisObjective::addObjective(HebiIKPtr ik) const
{
  return hebiIKAddObjectiveEndEffectorTipAxis(ik, _weight, 0, _x, _y, _z);
}

JointLimitConstraint::JointLimitConstraint(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions)
  : _weight(1.0f), _min_positions(min_positions), _max_positions(max_positions)
{ } 

JointLimitConstraint::JointLimitConstraint(double weight, const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions)
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

////////////////////////// RobotModel

void RobotModel::setTranslate(Eigen::Matrix4d& matrix, double x, double y, double z)
{
  matrix(0, 3) = x;
  matrix(1, 3) = y;
  matrix(2, 3) = z;
}

void RobotModel::setRotX(Eigen::Matrix4d& matrix, double radians) {
  matrix(0, 0) = 1;
  matrix(0, 1) = 0;
  matrix(0, 2) = 0;
  matrix(1, 0) = 0;
  matrix(1, 1) = cos(radians);
  matrix(1, 2) = -sin(radians);
  matrix(2, 0) = 0;
  matrix(2, 1) = sin(radians);
  matrix(2, 2) = cos(radians);
}

void RobotModel::setSphereInertia(Eigen::VectorXd& inertia, double mass, double radius)
{
  if (inertia.size() != 6)
    inertia.resize(6);

  // Ixx = Iyy = Izz = 2mr^2 / 5
  // Ixy = Ixz = Iyz = 0;
  inertia[0] = inertia[1] = inertia[2] = 0.4f * mass * radius * radius;
  inertia[3] = inertia[4] = inertia[5] = 0;
}

void RobotModel::setRodXAxisInertia(Eigen::VectorXd& inertia, double mass, double length)
{
  if (inertia.size() != 6)
    inertia.resize(6);

  // Iyy = Izz = ml^2 / 12
  // Ixx = Ixy = Ixz = Iyz = 0;
  inertia[1] = inertia[2] = mass * length * length / 12.f;
  inertia[0] = inertia[3] = inertia[4] = inertia[5] = 0;
}

bool RobotModel::tryAdd(HebiRobotModelElementPtr element, bool combine)
{
  HebiStatusCode res = hebiRobotModelAdd(
    internal_, nullptr, 0, element, combine ? 1 : 0);
  if (res == HebiStatusFailure)
  {
    hebiRobotModelElementRelease(element);
    return false;
  }
  return true;
}

RobotModel::RobotModel()
  : internal_(hebiRobotModelCreate())
{
}

RobotModel::~RobotModel() noexcept
{
  hebiRobotModelRelease(internal_);
}

void RobotModel::setBaseFrame(const Eigen::Matrix4d& base_frame)
{
  // Put data into an array
  double transform[16];
  Map<Matrix<double, 4, 4, RowMajor> > tmp(transform);
  tmp = base_frame;
  hebiRobotModelSetBaseFrame(internal_, transform);
}

Eigen::Matrix4d RobotModel::getBaseFrame() const
{
  // Get the data into an array
  double transform[16];
  hebiRobotModelGetBaseFrame(internal_, transform);

  // Copy out data
  Map<const Matrix<double, 4, 4, RowMajor> > tmp(transform);
  Eigen::Matrix4d res;
  res = tmp;
  return res;
}

size_t RobotModel::getFrameCount(HebiFrameType frame_type) const
{
  return hebiRobotModelGetNumberOfFrames(internal_, frame_type);
}

size_t RobotModel::getDoFCount() const
{
  return hebiRobotModelGetNumberOfDoFs(internal_);
}

// TODO: handle trees/etc by passing in parent object here, and output index
bool RobotModel::addRigidBody(
  const Eigen::Matrix4d& com,
  const Eigen::VectorXd& inertia,
  double mass,
  const Eigen::Matrix4d& output,
  bool combine)
{
  if (inertia.size() != 6)
    return false;

  // Allocate double arrays for C interop:
  double com_array[16];
  double inertia_array[6];
  double output_array[16];

  // Convert the data:
  {
    Map<Matrix<double, 4, 4, RowMajor> > tmp(com_array);
    tmp = com;
  }
  {
    Map<Eigen::VectorXd> tmp(inertia_array, 6);
    tmp = inertia;
  }
  {
    Map<Matrix<double, 4, 4, RowMajor> > tmp(output_array);
    tmp = output;
  }

  HebiRobotModelElementPtr body = hebiRobotModelElementCreateRigidBody(
    com_array, inertia_array, mass, 1, output_array);

  return tryAdd(body, combine);
}

// TODO: handle trees/etc by passing in parent object here, and output index
bool RobotModel::addJoint(
  HebiJointType joint_type,
  bool combine)
{
  return tryAdd(hebiRobotModelElementCreateJoint(joint_type), combine);
}

bool RobotModel::addActuator(ActuatorType actuator_type)
{
  Matrix4d com = Matrix4d::Identity();

  Matrix4d input_to_axis = Matrix4d::Identity();

  double mass = actuator_masses_.at(actuator_type);
  HebiJointType joint_type = HebiJointTypeRotationZ;

  VectorXd inertia(6);
  if (actuator_type == ActuatorType::X5_1 ||
      actuator_type == ActuatorType::X5_4 ||
      actuator_type == ActuatorType::X5_9)
  {
    setTranslate(com, -0.0142, -0.0031, 0.0165);
    setTranslate(input_to_axis, 0.0, 0.0, 0.03105);
    
    inertia << 0.00015, // XX
               0.000255, // YY
               0.000350, // ZZ
               0.0000341, // XY
               0.0000118, // XZ
               0.00000229; // YZ
  }
  else if (actuator_type == ActuatorType::X8_3 ||
           actuator_type == ActuatorType::X8_9 ||
           actuator_type == ActuatorType::X8_16)
  {
    setTranslate(com, -0.0145, -0.0031, 0.0242);
    setTranslate(input_to_axis, 0.0, 0.0, 0.0451);
    inertia << 0.000246, // XX
               0.000380, // YY
               0.000463, // ZZ
               0.0000444, // XY
               0.0000266, // XZ
               0.00000422; // YZ
  }
  else
  {
    return false;
  }

  ///////////////////////////////////////////////////////////////
  // Create the actuator (currently, this is X-series specific)

  // Create and add the body (and handle failure cases)
  if (!addRigidBody(com, inertia, mass, input_to_axis, false))
    return false;

  // Create and add the joint (and handle failure cases)
  // Note -- weird case if this fails, as we've already added the body...
  // leaves the kinematic object in an odd state.
  return addJoint(joint_type, true);
}

bool RobotModel::addLink(LinkType link_type, double extension, double twist)
{
  if (link_type != LinkType::X5)
    return false;

  Matrix4d com = Matrix4d::Identity();
  Matrix4d output = Matrix4d::Identity();

  // Tube approx. 0.4kg / 1m; 0.03 m shorter than the total extension length
  // End brackets + hardware approx. 0.26 kg
  double mass = 0.4f * (extension - 0.03) + 0.26f;

  // Edge of bracket to center of pipe.
  double edge_to_center = .0175f;

  // Note that this ignores the effect of the end brackets on moving the com
  // slightly off center.
  setTranslate(com, extension * 0.5f, 0, edge_to_center);
  setRotX(output, twist);
  setTranslate(output, extension,
               -edge_to_center * sin(twist),
               edge_to_center * (1 + cos(twist)));

  // Inertia: current approximation is a thin rod; will be improved in future.
  VectorXd inertia(6);
  setRodXAxisInertia(inertia, mass, extension);

  // Generic -- (try to) create and add the body
  return addRigidBody(com, inertia, mass, output, false);
}

bool RobotModel::addBracket(BracketType bracket_type)
{
  Matrix4d com = Matrix4d::Identity();
  Matrix4d output = Matrix4d::Identity();
  double mass = 0;

  // Light bracket
  switch(bracket_type)
  {
    case BracketType::X5LightLeft:
    case BracketType::X5LightRight:
    {
      double mult = 1;
      if (bracket_type == BracketType::X5LightRight)
        mult = -1;

      mass = 0.1;

      setTranslate(com, 0, mult * 0.0215f, 0.02f);
      setRotX(output, mult * (-M_PI / 2.0f));
      setTranslate(output, 0, mult * .043f, 0.04f);
      break;
    }
    case BracketType::X5HeavyLeftInside:
    case BracketType::X5HeavyLeftOutside:
    case BracketType::X5HeavyRightInside:
    case BracketType::X5HeavyRightOutside:
    {
      double lr_mult = 1;
      if (bracket_type == BracketType::X5HeavyRightInside ||
          bracket_type == BracketType::X5HeavyRightOutside)
        lr_mult = -1;

      double y_dist = -0.0225; // Inside
      if (bracket_type == BracketType::X5HeavyLeftOutside ||
          bracket_type == BracketType::X5HeavyRightOutside)
        y_dist = 0.0375; // Outside

      mass = 0.215;

      setTranslate(com, 0, lr_mult * 0.5 * y_dist, 0.0275);
      setRotX(output, lr_mult * (-M_PI / 2.0));
      setTranslate(output, 0, lr_mult * y_dist, 0.055);
      break;
    }
    default:
      return false;
  }

  // Inertia: current approximation is a 6cm sphere; will be improved in future.
  VectorXd inertia(6);
  setSphereInertia(inertia, mass, 0.06);

  // Generic -- (try to) create and add the body
  return addRigidBody(com, inertia, mass, output, false);
}

void RobotModel::getForwardKinematics(HebiFrameType frame_type, const Eigen::VectorXd& positions, Matrix4dVector& frames) const
{
  getFK(frame_type, positions, frames);
}

void RobotModel::getFK(HebiFrameType frame_type, const Eigen::VectorXd& positions, Matrix4dVector& frames) const
{
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }
  size_t num_frames = getFrameCount(frame_type);
  auto frame_array = new double[16 * num_frames];
  // Get data from C API
  hebiRobotModelGetForwardKinematics(internal_, frame_type, positions_array, frame_array);
  delete[] positions_array;
  // Copy into vector of matrices passed in
  frames.resize(num_frames);
  for (size_t i = 0; i < num_frames; ++i)
  {
    Map<Matrix<double, 4, 4, RowMajor> > tmp(frame_array + i * 16);
    frames[i] = tmp;
  }
  delete[] frame_array;
}

void RobotModel::getEndEffector(const Eigen::VectorXd& positions, Eigen::Matrix4d& transform) const
{
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }

  double transform_array[16];
  hebiRobotModelGetForwardKinematics(internal_, HebiFrameTypeEndEffector, positions_array, transform_array);
  delete[] positions_array;
  {
    Map<Matrix<double, 4, 4, RowMajor> > tmp(transform_array);
    transform = tmp;
  }
}

void RobotModel::getJacobians(HebiFrameType frame_type, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const
{
  getJ(frame_type, positions, jacobians);
}
void RobotModel::getJ(HebiFrameType frame_type, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const
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
  auto jacobians_array = new double[rows * cols];
  hebiRobotModelGetJacobians(internal_, frame_type, positions_array, jacobians_array);
  delete[] positions_array;
  jacobians.resize(num_frames);
  for (size_t i = 0; i < num_frames; ++i)
  {
    Map<Matrix<double, Dynamic, Dynamic, RowMajor> > tmp(jacobians_array + i * cols * 6, 6, cols);
    jacobians[i] = tmp;
  }
  delete[] jacobians_array;
}
void RobotModel::getJacobianEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const
{
  getJEndEffector(positions, jacobian);
}
void RobotModel::getJEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const
{
  MatrixXdVector tmp_jacobians;
  getJacobians(HebiFrameTypeEndEffector, positions, tmp_jacobians);

  // NOTE: could make this more efficient by writing additional lib function
  // for this, instead of tossing away almost everything from the full one!

  size_t num_dofs = positions.size();
  jacobian.resize(6, num_dofs);
  jacobian = *tmp_jacobians.rbegin();
}

void RobotModel::getMasses(Eigen::VectorXd& masses) const
{
  size_t num_masses = getFrameCount(HebiFrameTypeCenterOfMass);
  auto masses_array = new double[num_masses];
  hebiRobotModelGetMasses(internal_, masses_array);
  {
    Map<VectorXd> tmp(masses_array, num_masses);
    masses = tmp;
  }
  delete[] masses_array;
}

const std::map<RobotModel::ActuatorType, double> RobotModel::actuator_masses_  =  {
  std::make_pair(ActuatorType::X5_1, 0.315),
  std::make_pair(ActuatorType::X5_4, 0.335),
  std::make_pair(ActuatorType::X5_9, 0.360),
  std::make_pair(ActuatorType::X8_3, 0.460),
  std::make_pair(ActuatorType::X8_9, 0.480),
  std::make_pair(ActuatorType::X8_16, 0.500)
};

} // namespace robot_model
} // namespace hebi
