#include "group_feedback.hpp"

namespace hebi {

GroupFeedback::GroupFeedback(int number_of_modules)
 : internal_(hebiGroupFeedbackCreate(number_of_modules)),
   manage_pointer_lifetime_(true),
   number_of_modules_(number_of_modules)
{
  for (int i = 0; i < number_of_modules_; i++)
    feedbacks_.emplace_back(hebiGroupFeedbackGetModuleFeedback(internal_, i));
}

GroupFeedback::GroupFeedback(HebiGroupFeedbackPtr group_feedback)
 : internal_(group_feedback),
   manage_pointer_lifetime_(false),
   number_of_modules_(hebiGroupFeedbackGetSize(group_feedback))
{
  for (int i = 0; i < number_of_modules_; i++)
    feedbacks_.emplace_back(hebiGroupFeedbackGetModuleFeedback(internal_, i));
}

GroupFeedback::~GroupFeedback() noexcept
{
  if (manage_pointer_lifetime_ && internal_ != nullptr)
    hebiGroupFeedbackRelease(internal_);
}

int GroupFeedback::size() const
{
  return number_of_modules_;
}

const Feedback& GroupFeedback::operator[](int index) const
{
  return feedbacks_[index];
}

Eigen::VectorXd GroupFeedback::getBoardTemperature() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].boardTemperature();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getProcessorTemperature() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].processorTemperature();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getVoltage() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].voltage();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

Eigen::VectorXd GroupFeedback::getDeflection() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().deflection();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getDeflectionVelocity() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().deflectionVelocity();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getMotorVelocity() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().motorVelocity();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getMotorCurrent() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().motorCurrent();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getMotorSensorTemperature() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().motorSensorTemperature();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getMotorWindingCurrent() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().motorWindingCurrent();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

Eigen::VectorXd GroupFeedback::getMotorWindingTemperature() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().motorWindingTemperature();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getMotorHousingTemperature() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().motorHousingTemperature();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

Eigen::VectorXd GroupFeedback::getPosition() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().position();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getVelocity() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().velocity();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getEffort() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().effort();
    res[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

Eigen::VectorXd GroupFeedback::getPositionCommand() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().positionCommand();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getVelocityCommand() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().velocityCommand();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupFeedback::getEffortCommand() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().effortCommand();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

Eigen::MatrixX3d GroupFeedback::getAccelerometer() const
{
  Eigen::MatrixX3d res(number_of_modules_, 3);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].imu().accelerometer();
    if (fbk)
    {
      auto vec = fbk.get();
      res(i, 0) = vec.getX();
      res(i, 1) = vec.getY();
      res(i, 2) = vec.getZ();
    }
    else
    {
      res(i, 0) = std::numeric_limits<double>::quiet_NaN();
      res(i, 1) = std::numeric_limits<double>::quiet_NaN();
      res(i, 2) = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return res;
}
Eigen::MatrixX3d GroupFeedback::getGyro() const
{
  Eigen::MatrixX3d res(number_of_modules_, 3);
  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].imu().gyro();
    if (fbk)
    {
      auto vec = fbk.get();
      res(i, 0) = vec.getX();
      res(i, 1) = vec.getY();
      res(i, 2) = vec.getZ();
    }
    else
    {
      res(i, 0) = std::numeric_limits<double>::quiet_NaN();
      res(i, 1) = std::numeric_limits<double>::quiet_NaN();
      res(i, 2) = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return res;
}

void GroupFeedback::getBoardTemperature(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].boardTemperature();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getProcessorTemperature(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].processorTemperature();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getVoltage(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].voltage();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

void GroupFeedback::getDeflection(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().deflection();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getDeflectionVelocity(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().deflectionVelocity();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getMotorVelocity(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().motorVelocity();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getMotorCurrent(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().motorCurrent();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getMotorSensorTemperature(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().motorSensorTemperature();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getMotorWindingCurrent(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().motorWindingCurrent();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

void GroupFeedback::getMotorWindingTemperature(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().motorWindingTemperature();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getMotorHousingTemperature(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().motorHousingTemperature();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

void GroupFeedback::getPosition(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().position();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getVelocity(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().velocity();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getEffort(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].actuator().effort();
    out[i] = (fbk) ? fbk.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

void GroupFeedback::getPositionCommand(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().positionCommand();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getVelocityCommand(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().velocityCommand();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupFeedback::getEffortCommand(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& cmd = feedbacks_[i].actuator().effortCommand();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

void GroupFeedback::getAccelerometer(Eigen::MatrixX3d& out) const
{
  if (out.rows() != number_of_modules_ || out.cols() != 3)
  {
    out.resize(number_of_modules_, 3);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].imu().accelerometer();
    if (fbk)
    {
      auto vec = fbk.get();
      out(i, 0) = vec.getX();
      out(i, 1) = vec.getY();
      out(i, 2) = vec.getZ();
    }
    else
    {
      out(i, 0) = std::numeric_limits<double>::quiet_NaN();
      out(i, 1) = std::numeric_limits<double>::quiet_NaN();
      out(i, 2) = std::numeric_limits<double>::quiet_NaN();
    }
  }
}
void GroupFeedback::getGyro(Eigen::MatrixX3d& out) const
{
  if (out.rows() != number_of_modules_ || out.cols() != 3)
  {
    out.resize(number_of_modules_, 3);
  }

  for (int i = 0; i < number_of_modules_; ++i)
  {
    auto& fbk = feedbacks_[i].imu().gyro();
    if (fbk)
    {
      auto vec = fbk.get();
      out(i, 0) = vec.getX();
      out(i, 1) = vec.getY();
      out(i, 2) = vec.getZ();
    }
    else
    {
      out(i, 0) = std::numeric_limits<double>::quiet_NaN();
      out(i, 1) = std::numeric_limits<double>::quiet_NaN();
      out(i, 2) = std::numeric_limits<double>::quiet_NaN();
    }
  }
}

} // namespace hebi
