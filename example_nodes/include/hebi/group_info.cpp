#include "group_info.hpp"

namespace hebi {

GroupInfo::GroupInfo(size_t number_of_modules)
 : internal_(hebiGroupInfoCreate(number_of_modules)),
   number_of_modules_(number_of_modules)
{
  for (size_t i = 0; i < number_of_modules_; i++)
    infos_.emplace_back(hebiGroupInfoGetModuleInfo(internal_, i));
}

GroupInfo::~GroupInfo() noexcept
{
  if (internal_ != nullptr)
    hebiGroupInfoRelease(internal_);
}

size_t GroupInfo::size() const
{
  return number_of_modules_;
}

const Info& GroupInfo::operator[](size_t index) const
{
  return infos_[index];
}

bool GroupInfo::writeGains(const std::string& file) const
{
  return hebiGroupInfoWriteGains(internal_, file.c_str()) == HebiStatusSuccess;
}

Eigen::VectorXd GroupInfo::getSpringConstant() const
{
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    auto& info = infos_[i].settings().actuator().springConstant();
    res[i] = (info) ? info.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

void GroupInfo::getSpringConstant(Eigen::VectorXd& out) const
{
  if (out.size() != number_of_modules_)
  {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i)
  {
    auto& info = infos_[i].settings().actuator().springConstant();
    out[i] = (info) ? info.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

} // namespace hebi
