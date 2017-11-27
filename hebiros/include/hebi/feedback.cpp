#include "feedback.hpp"
#include <cmath>
#include <limits>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hebi {

Feedback::FloatField::FloatField(HebiFeedbackPtr internal, HebiFeedbackFloatField field)
  : internal_(internal), field_(field)
{
}

Feedback::FloatField::operator bool() const
{
  return has();
}

bool Feedback::FloatField::has() const
{
  return (hebiFeedbackGetFloat(internal_, field_, nullptr) == HebiStatusSuccess);
}

float Feedback::FloatField::get() const
{
  float ret;
  if (hebiFeedbackGetFloat(internal_, field_, &ret) != HebiStatusSuccess)
  {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Feedback::HighResAngleField::HighResAngleField(HebiFeedbackPtr internal, HebiFeedbackHighResAngleField field)
  : internal_(internal), field_(field)
{
}

Feedback::HighResAngleField::operator bool() const
{
  return has();
}

bool Feedback::HighResAngleField::has() const
{
  return (hebiFeedbackGetHighResAngle(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

double Feedback::HighResAngleField::get() const
{
  int64_t revolutions;
  float radian_offset;
  if (hebiFeedbackGetHighResAngle(internal_, field_, &revolutions, &radian_offset) != HebiStatusSuccess)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (
    static_cast<double>(revolutions) * 2.0 * M_PI +
    static_cast<double>(radian_offset)
  );
}

void Feedback::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const
{
  if (hebiFeedbackGetHighResAngle(internal_, field_, revolutions, radian_offset) != HebiStatusSuccess)
  {
    *revolutions = 0;
    *radian_offset = std::numeric_limits<float>::quiet_NaN();
  }
}

Feedback::NumberedFloatField::NumberedFloatField(HebiFeedbackPtr internal, HebiFeedbackNumberedFloatField field)
  : internal_(internal), field_(field)
{
}

bool Feedback::NumberedFloatField::has(int fieldNumber) const
{
  return (hebiFeedbackGetNumberedFloat(internal_, field_, fieldNumber, nullptr) == HebiStatusSuccess);
}

float Feedback::NumberedFloatField::get(int fieldNumber) const
{
  float ret;
  if (hebiFeedbackGetNumberedFloat(internal_, field_, fieldNumber, &ret) != HebiStatusSuccess)
  {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Feedback::UInt64Field::UInt64Field(HebiFeedbackPtr internal, HebiFeedbackUInt64Field field)
  : internal_(internal), field_(field)
{
}

Feedback::UInt64Field::operator bool() const
{
  return has();
}

bool Feedback::UInt64Field::has() const
{
  return (hebiFeedbackGetUInt64(internal_, field_, nullptr) == HebiStatusSuccess);
}

uint64_t Feedback::UInt64Field::get() const
{
  uint64_t ret;
  if (hebiFeedbackGetUInt64(internal_, field_, &ret) != HebiStatusSuccess)
  {
    ret = 0;
  }
  return ret;
}

Feedback::Vector3fField::Vector3fField(HebiFeedbackPtr internal, HebiFeedbackVector3fField field)
  : internal_(internal), field_(field)
{
}

Feedback::Vector3fField::operator bool() const
{
  return has();
}

bool Feedback::Vector3fField::has() const
{
  return (hebiFeedbackGetVector3f(internal_, field_, nullptr) == HebiStatusSuccess);
}

Vector3f Feedback::Vector3fField::get() const
{
  HebiVector3f ret;
  if (hebiFeedbackGetVector3f(internal_, field_, &ret) != HebiStatusSuccess)
  {
    ret.x = std::numeric_limits<float>::quiet_NaN();
    ret.y = std::numeric_limits<float>::quiet_NaN();
    ret.z = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Feedback::IoBank::IoBank(HebiFeedbackPtr internal, HebiFeedbackIoPinBank bank)
  : internal_(internal), bank_(bank)
{
}

bool Feedback::IoBank::hasInt(int pinNumber) const
{
  return (hebiFeedbackGetIoPinInt(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

bool Feedback::IoBank::hasFloat(int pinNumber) const
{
  return (hebiFeedbackGetIoPinFloat(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

int64_t Feedback::IoBank::getInt(int pinNumber) const
{
  int64_t ret;
  hebiFeedbackGetIoPinInt(internal_, bank_, pinNumber, &ret);
  return ret;
}

float Feedback::IoBank::getFloat(int pinNumber) const
{
  float ret;
  hebiFeedbackGetIoPinFloat(internal_, bank_, pinNumber, &ret);
  return ret;
}

Feedback::LedField::LedField(HebiFeedbackPtr internal, HebiFeedbackLedField field)
  : internal_(internal), field_(field)
{
}

bool Feedback::LedField::hasColor() const
{
  return (hebiFeedbackGetLedColor(internal_, field_, nullptr, nullptr, nullptr) == HebiStatusSuccess);
}

Color Feedback::LedField::getColor() const
{
  uint8_t r, g, b;
  if (hebiFeedbackGetLedColor(internal_, field_, &r, &g, &b) != HebiStatusSuccess)
  {
    r = 0;
    g = 0;
    b = 0;
  }
  return Color(r, g, b);
}

Feedback::Feedback(HebiFeedbackPtr feedback)
  : internal_(feedback),
    io_(internal_),
    actuator_(internal_),
    imu_(internal_),
    board_temperature_(internal_, HebiFeedbackFloatBoardTemperature),
    processor_temperature_(internal_, HebiFeedbackFloatProcessorTemperature),
    voltage_(internal_, HebiFeedbackFloatVoltage),
    debug_(internal_, HebiFeedbackNumberedFloatDebug),
    led_(internal_, HebiFeedbackLedLed)
{
}
Feedback::~Feedback() noexcept
{
}

Feedback::Feedback(Feedback&& other)
  : internal_(other.internal_),
    io_(internal_),
    actuator_(internal_),
    imu_(internal_),
    board_temperature_(internal_, HebiFeedbackFloatBoardTemperature),
    processor_temperature_(internal_, HebiFeedbackFloatProcessorTemperature),
    voltage_(internal_, HebiFeedbackFloatVoltage),
    debug_(internal_, HebiFeedbackNumberedFloatDebug),
    led_(internal_, HebiFeedbackLedLed)
{
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.
}

const Feedback::FloatField& Feedback::boardTemperature() const
{
  return board_temperature_;
}
const Feedback::FloatField& Feedback::processorTemperature() const
{
  return processor_temperature_;
}
const Feedback::FloatField& Feedback::voltage() const
{
  return voltage_;
}
const Feedback::NumberedFloatField& Feedback::debug() const
{
  return debug_;
}
const Feedback::LedField& Feedback::led() const
{
  return led_;
}

} // namespace hebi
