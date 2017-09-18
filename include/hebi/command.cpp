#include "command.hpp"
#include <cmath>
#include <limits>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hebi {

Command::FloatField::FloatField(HebiCommandPtr internal, HebiCommandFloatField field)
  : internal_(internal), field_(field)
{
}

Command::FloatField::operator bool() const
{
  return has();
}

bool Command::FloatField::has() const
{
  return (hebiCommandGetFloat(internal_, field_, nullptr) == HebiStatusSuccess);
}

float Command::FloatField::get() const
{
  float ret;
  if (hebiCommandGetFloat(internal_, field_, &ret) != HebiStatusSuccess)
  {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

void Command::FloatField::set(float value)
{
  hebiCommandSetFloat(internal_, field_, &value);
}

void Command::FloatField::clear()
{
  hebiCommandSetFloat(internal_, field_, nullptr);
}

Command::HighResAngleField::HighResAngleField(HebiCommandPtr internal, HebiCommandHighResAngleField field)
  : internal_(internal), field_(field)
{
}

Command::HighResAngleField::operator bool() const
{
  return has();
}

bool Command::HighResAngleField::has() const
{
  return (hebiCommandGetHighResAngle(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

double Command::HighResAngleField::get() const
{
  int64_t revolutions;
  float radian_offset;
  if (hebiCommandGetHighResAngle(internal_, field_, &revolutions, &radian_offset) != HebiStatusSuccess)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (
    static_cast<double>(revolutions) * 2.0 * M_PI +
    static_cast<double>(radian_offset)
  );
}

void Command::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const
{
  if (hebiCommandGetHighResAngle(internal_, field_, revolutions, radian_offset) != HebiStatusSuccess)
  {
    *revolutions = 0;
    *radian_offset = std::numeric_limits<float>::quiet_NaN();
  }
}

void Command::HighResAngleField::set(double radians)
{
  double revolutions_raw = radians / 2.0 / M_PI;
  double revolutions_int_d;
  double radian_offset_d = std::modf (revolutions_raw, &revolutions_int_d);
  radian_offset_d = radian_offset_d * 2.0 * M_PI;

  int64_t revolutions_int = std::isnan(revolutions_int_d) ? 0 : static_cast<int64_t>(revolutions_int_d);
  float radian_offset = static_cast<float>(radian_offset_d);
  hebiCommandSetHighResAngle(internal_, field_, &revolutions_int, &radian_offset);
}

void Command::HighResAngleField::set(int64_t revolutions, float radian_offset)
{
  hebiCommandSetHighResAngle(internal_, field_, &revolutions, &radian_offset);
}

void Command::HighResAngleField::clear()
{
  hebiCommandSetHighResAngle(internal_, field_, nullptr, nullptr);
}

Command::NumberedFloatField::NumberedFloatField(HebiCommandPtr internal, HebiCommandNumberedFloatField field)
  : internal_(internal), field_(field)
{
}

bool Command::NumberedFloatField::has(int fieldNumber) const
{
  return (hebiCommandGetNumberedFloat(internal_, field_, fieldNumber, nullptr) == HebiStatusSuccess);
}

float Command::NumberedFloatField::get(int fieldNumber) const
{
  float ret;
  if (hebiCommandGetNumberedFloat(internal_, field_, fieldNumber, &ret) != HebiStatusSuccess)
  {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

void Command::NumberedFloatField::set(int fieldNumber, float value)
{
  hebiCommandSetNumberedFloat(internal_, field_, fieldNumber, &value);
}

void Command::NumberedFloatField::clear(int fieldNumber)
{
  hebiCommandSetNumberedFloat(internal_, field_, fieldNumber, nullptr);
}

Command::BoolField::BoolField(HebiCommandPtr internal, HebiCommandBoolField field)
  : internal_(internal), field_(field)
{
}

bool Command::BoolField::has() const
{
  return (hebiCommandGetBool(internal_, field_, nullptr) == HebiStatusSuccess);
}

bool Command::BoolField::get() const
{
  int ret;
  if (hebiCommandGetBool(internal_, field_, &ret) != HebiStatusSuccess)
  {
    ret = 0;
  }
  return static_cast<bool>(ret);
}

void Command::BoolField::set(bool value)
{
  auto val = static_cast<int>(value);
  hebiCommandSetBool(internal_, field_, &val);
}

void Command::BoolField::clear()
{
  hebiCommandSetBool(internal_, field_, nullptr);
}

Command::StringField::StringField(HebiCommandPtr internal, HebiCommandStringField field)
  : internal_(internal), field_(field)
{
}

Command::StringField::operator bool() const
{
  return has();
}

bool Command::StringField::has() const
{
  return (hebiCommandGetString(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

std::string Command::StringField::get() const
{
  // Get the size first
  size_t length;
  if (hebiCommandGetString(internal_, field_, nullptr, &length) != HebiStatusSuccess)
  {
    // String field doesn't exist -- return an empty string
    return "";
  }
  auto buffer = new char [length];
  hebiCommandGetString(internal_, field_, buffer, &length);
  std::string tmp(buffer, length - 1);
  delete[] buffer;
  return tmp;
}

void Command::StringField::set(const std::string& value)
{
  const char* buffer = value.c_str();
  size_t length = value.size();
  hebiCommandSetString(internal_, field_, buffer, &length);
}

void Command::StringField::clear()
{
  hebiCommandSetString(internal_, field_, nullptr, nullptr);
}

Command::FlagField::FlagField(HebiCommandPtr internal, HebiCommandFlagField field)
  : internal_(internal), field_(field)
{
}

Command::FlagField::operator bool() const
{
  return has();
}

bool Command::FlagField::has() const
{
  return hebiCommandGetFlag(internal_, field_) == 1;
}

void Command::FlagField::set()
{
  hebiCommandSetFlag(internal_, field_, 1);
}

void Command::FlagField::clear()
{
  hebiCommandSetFlag(internal_, field_, 0);
}

Command::IoBank::IoBank(HebiCommandPtr internal, HebiCommandIoPinBank bank)
  : internal_(internal), bank_(bank)
{
}

bool Command::IoBank::hasInt(int pinNumber) const
{
  return (hebiCommandGetIoPinInt(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

bool Command::IoBank::hasFloat(int pinNumber) const
{
  return (hebiCommandGetIoPinFloat(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

int64_t Command::IoBank::getInt(int pinNumber) const
{
  int64_t ret;
  hebiCommandGetIoPinInt(internal_, bank_, pinNumber, &ret);
  return ret;
}

float Command::IoBank::getFloat(int pinNumber) const
{
  float ret;
  hebiCommandGetIoPinFloat(internal_, bank_, pinNumber, &ret);
  return ret;
}

void Command::IoBank::setInt(int pinNumber, int64_t value)
{
  hebiCommandSetIoPinInt(internal_, bank_, pinNumber, &value);
}

void Command::IoBank::setFloat(int pinNumber, float value)
{
  hebiCommandSetIoPinFloat(internal_, bank_, pinNumber, &value);
}

void Command::IoBank::clear(int pinNumber)
{
  hebiCommandSetIoPinInt(internal_, bank_, pinNumber, nullptr);
  hebiCommandSetIoPinFloat(internal_, bank_, pinNumber, nullptr);
}

Command::LedField::LedField(HebiCommandPtr internal, HebiCommandLedField field)
  : internal_(internal), field_(field)
{
}

bool Command::LedField::hasColor() const
{
  return (hebiCommandGetLedColor(internal_, field_, nullptr, nullptr, nullptr) == HebiStatusSuccess);
}

bool Command::LedField::hasModuleControl() const
{
  return (hebiCommandHasLedModuleControl(internal_, field_) == 1);
}

Color Command::LedField::getColor() const
{
  uint8_t r, g, b;
  hebiCommandGetLedColor(internal_, field_, &r, &g, &b);
  return Color(r, g, b);
}

void Command::LedField::setOverrideColor(const Color& new_color)
{
  hebiCommandSetLedOverrideColor(internal_, field_,
    new_color.getRed(), new_color.getGreen(), new_color.getBlue());
}

void Command::LedField::setModuleControl()
{
  hebiCommandSetLedModuleControl(internal_, field_);
}

void Command::LedField::clear()
{
  hebiCommandClearLed(internal_, field_);
}

Command::Command(HebiCommandPtr command)
  : internal_(command),
    io_(internal_),
    settings_(internal_),
    actuator_(internal_),
    debug_(internal_, HebiCommandNumberedFloatDebug),
    led_(internal_, HebiCommandLedLed)
{
}
Command::~Command() noexcept
{
}

Command::Command(Command&& other)
  : internal_(other.internal_),
    io_(internal_),
    settings_(internal_),
    actuator_(internal_),
    debug_(internal_, HebiCommandNumberedFloatDebug),
    led_(internal_, HebiCommandLedLed)
{
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.
}

Command::NumberedFloatField& Command::debug()
{
  return debug_;
}
Command::LedField& Command::led()
{
  return led_;
}

} // namespace hebi
