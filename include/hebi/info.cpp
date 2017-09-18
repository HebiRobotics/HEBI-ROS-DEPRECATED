#include "info.hpp"
#include <limits>

namespace hebi {

Info::FloatField::FloatField(HebiInfoPtr internal, HebiInfoFloatField field)
  : internal_(internal), field_(field)
{
}

Info::FloatField::operator bool() const
{
  return has();
}

bool Info::FloatField::has() const
{
  return (hebiInfoGetFloat(internal_, field_, nullptr) == HebiStatusSuccess);
}

float Info::FloatField::get() const
{
  float ret;
  if (hebiInfoGetFloat(internal_, field_, &ret) != HebiStatusSuccess)
  {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Info::BoolField::BoolField(HebiInfoPtr internal, HebiInfoBoolField field)
  : internal_(internal), field_(field)
{
}

bool Info::BoolField::has() const
{
  return (hebiInfoGetBool(internal_, field_, nullptr) == HebiStatusSuccess);
}

bool Info::BoolField::get() const
{
  int ret;
  if (hebiInfoGetBool(internal_, field_, &ret) != HebiStatusSuccess)
  {
    ret = 0;
  }
  return static_cast<bool>(ret);
}

Info::StringField::StringField(HebiInfoPtr internal, HebiInfoStringField field)
  : internal_(internal), field_(field)
{
}

Info::StringField::operator bool() const
{
  return has();
}

bool Info::StringField::has() const
{
  return (hebiInfoGetString(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

std::string Info::StringField::get() const
{
  // Get the size first
  size_t length;
  if (hebiInfoGetString(internal_, field_, nullptr, &length) != HebiStatusSuccess)
  {
    // String field doesn't exist -- return an empty string
    return "";
  }
  auto buffer = new char [length];
  hebiInfoGetString(internal_, field_, buffer, &length);
  std::string tmp(buffer, length - 1);
  delete[] buffer;
  return tmp;
}

Info::FlagField::FlagField(HebiInfoPtr internal, HebiInfoFlagField field)
  : internal_(internal), field_(field)
{
}

Info::FlagField::operator bool() const
{
  return has();
}

bool Info::FlagField::has() const
{
  return (hebiInfoGetFlag(internal_, field_) == 1);
}

Info::LedField::LedField(HebiInfoPtr internal, HebiInfoLedField field)
  : internal_(internal), field_(field)
{
}

bool Info::LedField::hasColor() const
{
  return (hebiInfoGetLedColor(internal_, field_, nullptr, nullptr, nullptr) == HebiStatusSuccess);
}

Color Info::LedField::getColor() const
{
  uint8_t r, g, b;
  if (hebiInfoGetLedColor(internal_, field_, &r, &g, &b) != HebiStatusSuccess)
  {
    r = 0;
    g = 0;
    b = 0;
  }
  return Color(r, g, b);
}

Info::Info(HebiInfoPtr info)
  : internal_(info),
    settings_(internal_),
    led_(internal_, HebiInfoLedLed)
{
}
Info::~Info() noexcept
{
}

Info::Info(Info&& other)
  : internal_(other.internal_),
    settings_(internal_),
    led_(internal_, HebiInfoLedLed)
{
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.
}

const Info::LedField& Info::led() const
{
  return led_;
}

} // namespace hebi
