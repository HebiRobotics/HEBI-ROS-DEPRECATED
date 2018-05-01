#include "group_command.hpp"

namespace hebi {

GroupCommand::GroupCommand(size_t number_of_modules)
 : internal_(hebiGroupCommandCreate(number_of_modules)),
   number_of_modules_(number_of_modules)
{
  for (size_t i = 0; i < number_of_modules_; i++)
    commands_.emplace_back(hebiGroupCommandGetModuleCommand(internal_, i));
}

GroupCommand::~GroupCommand() noexcept
{
  if (internal_ != nullptr)
    hebiGroupCommandRelease(internal_);
}

size_t GroupCommand::size() const
{
  return number_of_modules_;
}

Command& GroupCommand::operator[](size_t index)
{
  return commands_[index];
}

const Command& GroupCommand::operator[](size_t index) const
{
  return commands_[index];
}

bool GroupCommand::readGains(const std::string& file)
{
  return hebiGroupCommandReadGains(internal_, file.c_str()) == HebiStatusSuccess;
}

bool GroupCommand::writeGains(const std::string& file) const
{
  return hebiGroupCommandWriteGains(internal_, file.c_str()) == HebiStatusSuccess;
}

void GroupCommand::setPosition(const Eigen::VectorXd& position)
{
  if (position.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].actuator().position().set(position[i]);
}
void GroupCommand::setVelocity(const Eigen::VectorXd& velocity)
{
  if (velocity.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].actuator().velocity().set(static_cast<float>(velocity[i]));
}
void GroupCommand::setEffort(const Eigen::VectorXd& effort)
{
  if (effort.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].actuator().effort().set(static_cast<float>(effort[i]));
}

void GroupCommand::setSpringConstant(const Eigen::VectorXd& springConstant)
{
  if (springConstant.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].settings().actuator().springConstant().set(static_cast<float>(springConstant[i]));
}

Eigen::VectorXd GroupCommand::getPosition() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].actuator().position();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupCommand::getVelocity() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].actuator().velocity();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupCommand::getEffort() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].actuator().effort();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupCommand::getSpringConstant() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].settings().actuator().springConstant();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

void GroupCommand::getPosition(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].actuator().position();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupCommand::getVelocity(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].actuator().velocity();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupCommand::getEffort(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].actuator().effort();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupCommand::getSpringConstant(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    const auto& cmd = commands_[i].settings().actuator().springConstant();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

} // namespace hebi
