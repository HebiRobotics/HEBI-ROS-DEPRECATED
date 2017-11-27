#pragma once

#include "hebi.h"
#include "color.hpp"
#include <string>
#include "util.hpp"

namespace hebi {

/// \brief Command objects have various fields that can be set; when sent to the
/// module, these fields control internal properties and setpoints.
/// 
/// This object has a hierarchical structure -- there are some direct general-purpose
/// fields at the top level, and many more specific fields contained in different
/// nested subobjects.
/// 
/// The subobjects contain references to the parent command object, and so should not
/// be used after the parent object has been destroyed.
/// 
/// The fields in the command object are typed; generally, these are optional-style
/// read/write fields (i.e., have the concept of get/set/has/clear), although the
/// return types and exact interface vary slightly between fields. Where appropriate,
/// the explicit bool operator has been overridden so that you can shortcut
/// @c if(field.has()) by calling @c if(field).
/// 
/// Although this header file can be used to look at the hierarchy of the messages,
/// in general the online documentation at apidocs.hebi.us presents this information.
/// in a more readable form.
class Command final
{
  public:
    enum ControlStrategy {
      /// The motor is not given power (equivalent to a 0 PWM value)
      Off,
      /// A direct PWM value (-1 to 1) can be sent to the motor (subject to onboard safety limiting).
      DirectPWM,
      /// A combination of the position, velocity, and effort loops with P and V feeding to T; documented on docs.hebi.us under "Control Modes"
      Strategy2,
      /// A combination of the position, velocity, and effort loops with P, V, and T feeding to PWM; documented on docs.hebi.us under "Control Modes"
      Strategy3,
      /// A combination of the position, velocity, and effort loops with P feeding to T and V feeding to PWM; documented on docs.hebi.us under "Control Modes"
      Strategy4,
    };

  // Note: this is 'protected' instead of 'private' for easier use with Doxygen
  protected:
    /// \brief A message field representable by a single-precision floating point value.
    class FloatField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FloatField(HebiCommandPtr internal, HebiCommandFloatField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::FloatField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns a default.
        float get() const;
        /// \brief Sets the field to a given value.
        void set(float value);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        HebiCommandFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(FloatField)
    };
    /// \brief A message field for an angle measurement which does not lose
    /// precision at very high angles.
    ///
    /// This field is represented as an int64_t for the number of revolutions
    /// and a float for the radian offset from that number of revolutions.
    class HighResAngleField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        HighResAngleField(HebiCommandPtr internal, HebiCommandHighResAngleField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::HighResAngleField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value as a double;
        /// otherwise, returns a default.
        ///
        /// Note that some precision might be lost converting to a double at
        /// very high number of revolutions.
        double get() const;
        /// \brief If the field has a value, returns that value in the int64
        /// and float parameters passed in; otherwise, returns a default.
        ///
        /// Note that this maintains the full precision of the underlying angle
        /// measurement, even for very large numbers of revolutions.
        ///
        /// \param revolutions The number of full revolutions
        ///
        /// \param radian_offset The offset from the given number of full
        /// revolutions.  Note that this is usually between 0 and @c 2*M_PI, but
        /// callers should not assume this.
        void get(int64_t* revolutions, float* radian_offset) const;
        /// \brief Sets the field to a given double value (in radians).  Note
        /// that double precision floating point numbers cannot represent the
        /// same angular resolution at very high magnitudes as they can at lower
        /// magnitudes.
        void set(double radians);
        /// \brief Sets the field to a given integer number of revolutions and
        /// a floating point offset from this number of revolutions.  The offset
        /// does not specifically need to fall within a certain range (i.e., can
        /// add more than a single revolution to the integer value), but should
        /// be kept relatively small (e.g., below 10,000) to avoid potential
        /// loss of precision.
        void set(int64_t revolutions, float radian_offset);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        HebiCommandHighResAngleField const field_;

        HEBI_DISABLE_COPY_MOVE(HighResAngleField)
    };

    /// \brief A message field containing a numbered set of single-precision
    /// floating point values.
    class NumberedFloatField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        NumberedFloatField(HebiCommandPtr internal, HebiCommandNumberedFloatField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief True if (and only if) the particular numbered subvalue of
        /// this field has a value.
        ///
        /// \param fieldNumber Which subvalue to check; valid values for
        /// fieldNumber depend on the field type.
        bool has(int fieldNumber) const;
        /// \brief If the particular numbered subvalue of this field has a
        /// value, returns that value; otherwise returns a default.
        ///
        /// \param fieldNumber Which subvalue to get; valid values for
        /// fieldNumber depend on the field type.
        float get(int fieldNumber) const;
        /// \brief Sets the particular numbered subvalue of this field to a
        /// given value.
        ///
        /// \param fieldNumber Which subvalue to set; valid values for
        /// fieldNumber depend on the field type.
        void set(int fieldNumber, float value);
        /// \brief Removes any currently set value for the numbered subvalue of
        /// this field.
        ///
        /// \param fieldNumber Which subvalue to clear; valid values for
        /// fieldNumber depend on the field type.
        void clear(int fieldNumber);

      private:
        HebiCommandPtr const internal_;
        HebiCommandNumberedFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(NumberedFloatField)
    };

    /// \brief A message field representable by a bool value.
    class BoolField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        BoolField(HebiCommandPtr internal, HebiCommandBoolField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns false.
        bool get() const;
        /// \brief Sets the field to a given value.
        void set(bool value);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        HebiCommandBoolField const field_;

        HEBI_DISABLE_COPY_MOVE(BoolField)
    };

    /// \brief A message field representable by a std::string.
    class StringField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        StringField(HebiCommandPtr internal, HebiCommandStringField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::StringField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns a copy of that value;
        /// otherwise, returns a default.
        std::string get() const;
        /// \brief Sets the field to a given value.
        void set(const std::string& value);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        HebiCommandStringField const field_;

        HEBI_DISABLE_COPY_MOVE(StringField)
    };

    /// \brief A two-state message field (either set/true or cleared/false).
    class FlagField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FlagField(HebiCommandPtr internal, HebiCommandFlagField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the flag is set without
        /// directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::FlagField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief Returns @c true if the flag is set, false if it is cleared.
        bool has() const;
        /// \brief Sets this flag.
        void set();
        /// \brief Clears this flag (e.g., sets it to false/off).
        void clear();

      private:
        HebiCommandPtr const internal_;
        HebiCommandFlagField const field_;

        HEBI_DISABLE_COPY_MOVE(FlagField)
    };

    /// \brief A message field representable by an enum of a given type.
    template <class T>
    class EnumField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        EnumField(HebiCommandPtr internal, HebiCommandEnumField field)
          : internal_(internal), field_(field) {}
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::EnumField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const { return has(); }
        /// \brief True if (and only if) the field has a value.
        bool has() const { return (hebiCommandGetEnum(internal_, field_, nullptr) == HebiStatusSuccess); }
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns a default.
        T get() const { T ret{}; hebiCommandGetEnum(internal_, field_, reinterpret_cast<int*>(&ret)); return ret; }
        /// \brief Sets the field to a given value.
        void set(T value) { hebiCommandSetEnum(internal_, field_, reinterpret_cast<int*>(&value)); }
        /// \brief Removes any currently set value for this field.
        void clear() { hebiCommandSetEnum(internal_, field_, nullptr); }

      private:
        HebiCommandPtr const internal_;
        HebiCommandEnumField const field_;

        HEBI_DISABLE_COPY_MOVE(EnumField)
    };

    /// \brief A message field for interfacing with a bank of I/O pins.
    class IoBank final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        IoBank(HebiCommandPtr internal, HebiCommandIoPinBank bank);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief True if (and only if) the particular numbered pin in this
        /// bank has an integer (e.g., digital) value.
        ///
        /// \param pinNumber Which pin to check; valid values for pinNumber
        /// depend on the bank.
        bool hasInt(int pinNumber) const;
        /// \brief True if (and only if) the particular numbered pin in this
        /// bank has an floating point (e.g., analog or PWM) value.
        ///
        /// \param pinNumber Which pin to check; valid values for pinNumber
        /// depend on the bank.
        bool hasFloat(int pinNumber) const;
        /// \brief If this numbered pin in this bank has an integer (e.g.,
        /// digital) value, returns that value; otherwise returns a default.
        ///
        /// \param pinNumber Which pin to get; valid values for pinNumber
        /// depend on the bank.
        int64_t getInt(int pinNumber) const;
        /// \brief If this numbered pin in this bank has an floating point
        /// (e.g., analog or PWM) value, returns that value; otherwise returns a
        /// default.
        ///
        /// \param pinNumber Which pin to get; valid values for pinNumber
        /// depend on the bank.
        float getFloat(int pinNumber) const;
        /// \brief Sets the particular pin to a integer value (representing a
        /// digital output).
        ///
        /// \param pinNumber Which pin to set; valid values for pinNumber
        /// depend on the bank.
        void setInt(int pinNumber, int64_t value);
        /// \brief Sets the particular pin to a floating point value
        /// (representing a PWM output).
        ///
        /// \param pinNumber Which pin to set; valid values for pinNumber
        /// depend on the bank.
        void setFloat(int pinNumber, float value);
        /// \brief Removes any currently set value for this pin.
        ///
        /// \param pinNumber Which pin to clear; valid values for pinNumber
        /// depend on the bank.
        void clear(int pinNumber);

      private:
        HebiCommandPtr const internal_;
        HebiCommandIoPinBank const bank_;

        HEBI_DISABLE_COPY_MOVE(IoBank);
    };
    /// \brief A message field for interfacing with an LED.
    class LedField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        LedField(HebiCommandPtr internal, HebiCommandLedField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Returns true if the LED color is set, and false otherwise.
        bool hasColor() const;
        /// \brief Returns true if the message indicates that the module should
        /// resume control of the LED.
        ///
        /// Note: a return value of @c false can indicate either an override
        /// command (if and only if @c hasColor() returns @c true), or no
        /// information about the LED (i.e., the module should maintain it's
        /// current state regarding the LED).
        bool hasModuleControl() const;
        /// \brief Returns the led color.
        Color getColor() const;
        /// \brief Commands a color that overrides the module's control of the
        /// LED.
        void setOverrideColor(const Color& new_color);
        /// \brief Sets the module to regain control of the LED.
        void setModuleControl();
        /// \brief Removes any currently set value for this field, so that the
        /// module maintains its previous state of LED control/color (i.e., does
        /// not have an override color command or an explicit 'module control'
        /// command).
        void clear();

      private:
        HebiCommandPtr const internal_;
        HebiCommandLedField const field_;

        HEBI_DISABLE_COPY_MOVE(LedField)
    };

    /// Any available digital or analog output pins on the device.
    class Io final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Io(HebiCommandPtr internal)
          : internal_(internal),
            a_(internal, HebiCommandIoBankA),
            b_(internal, HebiCommandIoBankB),
            c_(internal, HebiCommandIoBankC),
            d_(internal, HebiCommandIoBankD),
            e_(internal, HebiCommandIoBankE),
            f_(internal, HebiCommandIoBankF)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Subfields ----------------
    
        /// I/O pin bank a (pins 1-8 available)
        IoBank& a() { return a_; }
        /// I/O pin bank a (pins 1-8 available)
        const IoBank& a() const { return a_; }
        /// I/O pin bank b (pins 1-8 available)
        IoBank& b() { return b_; }
        /// I/O pin bank b (pins 1-8 available)
        const IoBank& b() const { return b_; }
        /// I/O pin bank c (pins 1-8 available)
        IoBank& c() { return c_; }
        /// I/O pin bank c (pins 1-8 available)
        const IoBank& c() const { return c_; }
        /// I/O pin bank d (pins 1-8 available)
        IoBank& d() { return d_; }
        /// I/O pin bank d (pins 1-8 available)
        const IoBank& d() const { return d_; }
        /// I/O pin bank e (pins 1-8 available)
        IoBank& e() { return e_; }
        /// I/O pin bank e (pins 1-8 available)
        const IoBank& e() const { return e_; }
        /// I/O pin bank f (pins 1-8 available)
        IoBank& f() { return f_; }
        /// I/O pin bank f (pins 1-8 available)
        const IoBank& f() const { return f_; }
    
      private:
        HebiCommandPtr const internal_;
    
        IoBank a_;
        IoBank b_;
        IoBank c_;
        IoBank d_;
        IoBank e_;
        IoBank f_;
    
        HEBI_DISABLE_COPY_MOVE(Io)
    };

    /// Module settings that are typically changed at a slower rate.
    class Settings final
    {
      // Note: this is 'protected' instead of 'private' for easier use with Doxygen
      protected:
        /// Actuator-specific settings, such as controller gains.
        class Actuator final
        {
          // Note: this is 'protected' instead of 'private' for easier use with Doxygen
          protected:
            /// Controller gains for the position PID loop.
            class PositionGains final
            {
              public:
                #ifndef DOXYGEN_OMIT_INTERNAL
                PositionGains(HebiCommandPtr internal)
                  : internal_(internal),
                    position_kp_(internal, HebiCommandFloatPositionKp),
                    position_ki_(internal, HebiCommandFloatPositionKi),
                    position_kd_(internal, HebiCommandFloatPositionKd),
                    position_feed_forward_(internal, HebiCommandFloatPositionFeedForward),
                    position_dead_zone_(internal, HebiCommandFloatPositionDeadZone),
                    position_i_clamp_(internal, HebiCommandFloatPositionIClamp),
                    position_punch_(internal, HebiCommandFloatPositionPunch),
                    position_min_target_(internal, HebiCommandFloatPositionMinTarget),
                    position_max_target_(internal, HebiCommandFloatPositionMaxTarget),
                    position_target_lowpass_(internal, HebiCommandFloatPositionTargetLowpass),
                    position_min_output_(internal, HebiCommandFloatPositionMinOutput),
                    position_max_output_(internal, HebiCommandFloatPositionMaxOutput),
                    position_output_lowpass_(internal, HebiCommandFloatPositionOutputLowpass),
                    position_d_on_error_(internal, HebiCommandBoolPositionDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for position
                FloatField& positionKp() { return position_kp_; }
                /// Proportional PID gain for position
                const FloatField& positionKp() const { return position_kp_; }
                /// Integral PID gain for position
                FloatField& positionKi() { return position_ki_; }
                /// Integral PID gain for position
                const FloatField& positionKi() const { return position_ki_; }
                /// Derivative PID gain for position
                FloatField& positionKd() { return position_kd_; }
                /// Derivative PID gain for position
                const FloatField& positionKd() const { return position_kd_; }
                /// Feed forward term for position (this term is multiplied by the target and added to the output).
                FloatField& positionFeedForward() { return position_feed_forward_; }
                /// Feed forward term for position (this term is multiplied by the target and added to the output).
                const FloatField& positionFeedForward() const { return position_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                FloatField& positionDeadZone() { return position_dead_zone_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                const FloatField& positionDeadZone() const { return position_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                FloatField& positionIClamp() { return position_i_clamp_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                const FloatField& positionIClamp() const { return position_i_clamp_; }
                /// Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                FloatField& positionPunch() { return position_punch_; }
                /// Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                const FloatField& positionPunch() const { return position_punch_; }
                /// Minimum allowed value for input to the PID controller
                FloatField& positionMinTarget() { return position_min_target_; }
                /// Minimum allowed value for input to the PID controller
                const FloatField& positionMinTarget() const { return position_min_target_; }
                /// Maximum allowed value for input to the PID controller
                FloatField& positionMaxTarget() { return position_max_target_; }
                /// Maximum allowed value for input to the PID controller
                const FloatField& positionMaxTarget() const { return position_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& positionTargetLowpass() { return position_target_lowpass_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& positionTargetLowpass() const { return position_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                FloatField& positionMinOutput() { return position_min_output_; }
                /// Output from the PID controller is limited to a minimum of this value.
                const FloatField& positionMinOutput() const { return position_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                FloatField& positionMaxOutput() { return position_max_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                const FloatField& positionMaxOutput() const { return position_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& positionOutputLowpass() { return position_output_lowpass_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& positionOutputLowpass() const { return position_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                BoolField& positionDOnError() { return position_d_on_error_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                const BoolField& positionDOnError() const { return position_d_on_error_; }
            
              private:
                HebiCommandPtr const internal_;
            
                FloatField position_kp_;
                FloatField position_ki_;
                FloatField position_kd_;
                FloatField position_feed_forward_;
                FloatField position_dead_zone_;
                FloatField position_i_clamp_;
                FloatField position_punch_;
                FloatField position_min_target_;
                FloatField position_max_target_;
                FloatField position_target_lowpass_;
                FloatField position_min_output_;
                FloatField position_max_output_;
                FloatField position_output_lowpass_;
                BoolField position_d_on_error_;
            
                HEBI_DISABLE_COPY_MOVE(PositionGains)
            };
        
            /// Controller gains for the velocity PID loop.
            class VelocityGains final
            {
              public:
                #ifndef DOXYGEN_OMIT_INTERNAL
                VelocityGains(HebiCommandPtr internal)
                  : internal_(internal),
                    velocity_kp_(internal, HebiCommandFloatVelocityKp),
                    velocity_ki_(internal, HebiCommandFloatVelocityKi),
                    velocity_kd_(internal, HebiCommandFloatVelocityKd),
                    velocity_feed_forward_(internal, HebiCommandFloatVelocityFeedForward),
                    velocity_dead_zone_(internal, HebiCommandFloatVelocityDeadZone),
                    velocity_i_clamp_(internal, HebiCommandFloatVelocityIClamp),
                    velocity_punch_(internal, HebiCommandFloatVelocityPunch),
                    velocity_min_target_(internal, HebiCommandFloatVelocityMinTarget),
                    velocity_max_target_(internal, HebiCommandFloatVelocityMaxTarget),
                    velocity_target_lowpass_(internal, HebiCommandFloatVelocityTargetLowpass),
                    velocity_min_output_(internal, HebiCommandFloatVelocityMinOutput),
                    velocity_max_output_(internal, HebiCommandFloatVelocityMaxOutput),
                    velocity_output_lowpass_(internal, HebiCommandFloatVelocityOutputLowpass),
                    velocity_d_on_error_(internal, HebiCommandBoolVelocityDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for velocity
                FloatField& velocityKp() { return velocity_kp_; }
                /// Proportional PID gain for velocity
                const FloatField& velocityKp() const { return velocity_kp_; }
                /// Integral PID gain for velocity
                FloatField& velocityKi() { return velocity_ki_; }
                /// Integral PID gain for velocity
                const FloatField& velocityKi() const { return velocity_ki_; }
                /// Derivative PID gain for velocity
                FloatField& velocityKd() { return velocity_kd_; }
                /// Derivative PID gain for velocity
                const FloatField& velocityKd() const { return velocity_kd_; }
                /// Feed forward term for velocity (this term is multiplied by the target and added to the output).
                FloatField& velocityFeedForward() { return velocity_feed_forward_; }
                /// Feed forward term for velocity (this term is multiplied by the target and added to the output).
                const FloatField& velocityFeedForward() const { return velocity_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                FloatField& velocityDeadZone() { return velocity_dead_zone_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                const FloatField& velocityDeadZone() const { return velocity_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                FloatField& velocityIClamp() { return velocity_i_clamp_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                const FloatField& velocityIClamp() const { return velocity_i_clamp_; }
                /// Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                FloatField& velocityPunch() { return velocity_punch_; }
                /// Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                const FloatField& velocityPunch() const { return velocity_punch_; }
                /// Minimum allowed value for input to the PID controller
                FloatField& velocityMinTarget() { return velocity_min_target_; }
                /// Minimum allowed value for input to the PID controller
                const FloatField& velocityMinTarget() const { return velocity_min_target_; }
                /// Maximum allowed value for input to the PID controller
                FloatField& velocityMaxTarget() { return velocity_max_target_; }
                /// Maximum allowed value for input to the PID controller
                const FloatField& velocityMaxTarget() const { return velocity_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& velocityTargetLowpass() { return velocity_target_lowpass_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& velocityTargetLowpass() const { return velocity_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                FloatField& velocityMinOutput() { return velocity_min_output_; }
                /// Output from the PID controller is limited to a minimum of this value.
                const FloatField& velocityMinOutput() const { return velocity_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                FloatField& velocityMaxOutput() { return velocity_max_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                const FloatField& velocityMaxOutput() const { return velocity_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& velocityOutputLowpass() { return velocity_output_lowpass_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& velocityOutputLowpass() const { return velocity_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                BoolField& velocityDOnError() { return velocity_d_on_error_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                const BoolField& velocityDOnError() const { return velocity_d_on_error_; }
            
              private:
                HebiCommandPtr const internal_;
            
                FloatField velocity_kp_;
                FloatField velocity_ki_;
                FloatField velocity_kd_;
                FloatField velocity_feed_forward_;
                FloatField velocity_dead_zone_;
                FloatField velocity_i_clamp_;
                FloatField velocity_punch_;
                FloatField velocity_min_target_;
                FloatField velocity_max_target_;
                FloatField velocity_target_lowpass_;
                FloatField velocity_min_output_;
                FloatField velocity_max_output_;
                FloatField velocity_output_lowpass_;
                BoolField velocity_d_on_error_;
            
                HEBI_DISABLE_COPY_MOVE(VelocityGains)
            };
        
            /// Controller gains for the effort PID loop.
            class EffortGains final
            {
              public:
                #ifndef DOXYGEN_OMIT_INTERNAL
                EffortGains(HebiCommandPtr internal)
                  : internal_(internal),
                    effort_kp_(internal, HebiCommandFloatEffortKp),
                    effort_ki_(internal, HebiCommandFloatEffortKi),
                    effort_kd_(internal, HebiCommandFloatEffortKd),
                    effort_feed_forward_(internal, HebiCommandFloatEffortFeedForward),
                    effort_dead_zone_(internal, HebiCommandFloatEffortDeadZone),
                    effort_i_clamp_(internal, HebiCommandFloatEffortIClamp),
                    effort_punch_(internal, HebiCommandFloatEffortPunch),
                    effort_min_target_(internal, HebiCommandFloatEffortMinTarget),
                    effort_max_target_(internal, HebiCommandFloatEffortMaxTarget),
                    effort_target_lowpass_(internal, HebiCommandFloatEffortTargetLowpass),
                    effort_min_output_(internal, HebiCommandFloatEffortMinOutput),
                    effort_max_output_(internal, HebiCommandFloatEffortMaxOutput),
                    effort_output_lowpass_(internal, HebiCommandFloatEffortOutputLowpass),
                    effort_d_on_error_(internal, HebiCommandBoolEffortDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for effort
                FloatField& effortKp() { return effort_kp_; }
                /// Proportional PID gain for effort
                const FloatField& effortKp() const { return effort_kp_; }
                /// Integral PID gain for effort
                FloatField& effortKi() { return effort_ki_; }
                /// Integral PID gain for effort
                const FloatField& effortKi() const { return effort_ki_; }
                /// Derivative PID gain for effort
                FloatField& effortKd() { return effort_kd_; }
                /// Derivative PID gain for effort
                const FloatField& effortKd() const { return effort_kd_; }
                /// Feed forward term for effort (this term is multiplied by the target and added to the output).
                FloatField& effortFeedForward() { return effort_feed_forward_; }
                /// Feed forward term for effort (this term is multiplied by the target and added to the output).
                const FloatField& effortFeedForward() const { return effort_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                FloatField& effortDeadZone() { return effort_dead_zone_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                const FloatField& effortDeadZone() const { return effort_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                FloatField& effortIClamp() { return effort_i_clamp_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                const FloatField& effortIClamp() const { return effort_i_clamp_; }
                /// Constant offset to the effort PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                FloatField& effortPunch() { return effort_punch_; }
                /// Constant offset to the effort PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                const FloatField& effortPunch() const { return effort_punch_; }
                /// Minimum allowed value for input to the PID controller
                FloatField& effortMinTarget() { return effort_min_target_; }
                /// Minimum allowed value for input to the PID controller
                const FloatField& effortMinTarget() const { return effort_min_target_; }
                /// Maximum allowed value for input to the PID controller
                FloatField& effortMaxTarget() { return effort_max_target_; }
                /// Maximum allowed value for input to the PID controller
                const FloatField& effortMaxTarget() const { return effort_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& effortTargetLowpass() { return effort_target_lowpass_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& effortTargetLowpass() const { return effort_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                FloatField& effortMinOutput() { return effort_min_output_; }
                /// Output from the PID controller is limited to a minimum of this value.
                const FloatField& effortMinOutput() const { return effort_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                FloatField& effortMaxOutput() { return effort_max_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                const FloatField& effortMaxOutput() const { return effort_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& effortOutputLowpass() { return effort_output_lowpass_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& effortOutputLowpass() const { return effort_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                BoolField& effortDOnError() { return effort_d_on_error_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                const BoolField& effortDOnError() const { return effort_d_on_error_; }
            
              private:
                HebiCommandPtr const internal_;
            
                FloatField effort_kp_;
                FloatField effort_ki_;
                FloatField effort_kd_;
                FloatField effort_feed_forward_;
                FloatField effort_dead_zone_;
                FloatField effort_i_clamp_;
                FloatField effort_punch_;
                FloatField effort_min_target_;
                FloatField effort_max_target_;
                FloatField effort_target_lowpass_;
                FloatField effort_min_output_;
                FloatField effort_max_output_;
                FloatField effort_output_lowpass_;
                BoolField effort_d_on_error_;
            
                HEBI_DISABLE_COPY_MOVE(EffortGains)
            };
        
          public:
            #ifndef DOXYGEN_OMIT_INTERNAL
            Actuator(HebiCommandPtr internal)
              : internal_(internal),
                position_gains_(internal),
                velocity_gains_(internal),
                effort_gains_(internal),
                spring_constant_(internal, HebiCommandFloatSpringConstant),
                control_strategy_(internal, HebiCommandEnumControlStrategy)
            {
            }
            #endif // DOXYGEN_OMIT_INTERNAL
        
            // With all submessage and field getters: Note that the returned reference
            // should not be used after the lifetime of this parent.
        
            // Submessages ----------------
        
            /// Controller gains for the position PID loop.
            PositionGains& positionGains() { return position_gains_; }
            /// Controller gains for the position PID loop.
            const PositionGains& positionGains() const { return position_gains_; }
            /// Controller gains for the velocity PID loop.
            VelocityGains& velocityGains() { return velocity_gains_; }
            /// Controller gains for the velocity PID loop.
            const VelocityGains& velocityGains() const { return velocity_gains_; }
            /// Controller gains for the effort PID loop.
            EffortGains& effortGains() { return effort_gains_; }
            /// Controller gains for the effort PID loop.
            const EffortGains& effortGains() const { return effort_gains_; }
        
            // Subfields ----------------
        
            /// The spring constant of the module.
            FloatField& springConstant() { return spring_constant_; }
            /// The spring constant of the module.
            const FloatField& springConstant() const { return spring_constant_; }
            /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
            EnumField<ControlStrategy>& controlStrategy() { return control_strategy_; }
            /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
            const EnumField<ControlStrategy>& controlStrategy() const { return control_strategy_; }
        
          private:
            HebiCommandPtr const internal_;
        
            PositionGains position_gains_;
            VelocityGains velocity_gains_;
            EffortGains effort_gains_;
        
            FloatField spring_constant_;
            EnumField<ControlStrategy> control_strategy_;
        
            HEBI_DISABLE_COPY_MOVE(Actuator)
        };
    
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Settings(HebiCommandPtr internal)
          : internal_(internal),
            actuator_(internal),
            name_(internal, HebiCommandStringName),
            family_(internal, HebiCommandStringFamily),
            save_current_settings_(internal, HebiCommandFlagSaveCurrentSettings)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Submessages ----------------
    
        /// Actuator-specific settings, such as controller gains.
        Actuator& actuator() { return actuator_; }
        /// Actuator-specific settings, such as controller gains.
        const Actuator& actuator() const { return actuator_; }
    
        // Subfields ----------------
    
        /// Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
        StringField& name() { return name_; }
        /// Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
        const StringField& name() const { return name_; }
        /// Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
        StringField& family() { return family_; }
        /// Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
        const StringField& family() const { return family_; }
        /// Indicates if the module should save the current values of all of its settings.
        FlagField& saveCurrentSettings() { return save_current_settings_; }
        /// Indicates if the module should save the current values of all of its settings.
        const FlagField& saveCurrentSettings() const { return save_current_settings_; }
    
      private:
        HebiCommandPtr const internal_;
    
        Actuator actuator_;
    
        StringField name_;
        StringField family_;
        FlagField save_current_settings_;
    
        HEBI_DISABLE_COPY_MOVE(Settings)
    };

    /// Actuator-specific commands.
    class Actuator final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Actuator(HebiCommandPtr internal)
          : internal_(internal),
            velocity_(internal, HebiCommandFloatVelocity),
            effort_(internal, HebiCommandFloatEffort),
            position_(internal, HebiCommandHighResAnglePosition)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Subfields ----------------
    
        /// Velocity of the module output (post-spring), in radians/second.
        FloatField& velocity() { return velocity_; }
        /// Velocity of the module output (post-spring), in radians/second.
        const FloatField& velocity() const { return velocity_; }
        /// Effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
        FloatField& effort() { return effort_; }
        /// Effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
        const FloatField& effort() const { return effort_; }
        /// Position of the module output (post-spring), in radians.
        HighResAngleField& position() { return position_; }
        /// Position of the module output (post-spring), in radians.
        const HighResAngleField& position() const { return position_; }
    
      private:
        HebiCommandPtr const internal_;
    
        FloatField velocity_;
        FloatField effort_;
        HighResAngleField position_;
    
        HEBI_DISABLE_COPY_MOVE(Actuator)
    };

  private:
    /**
     * C-style object; managed by parent.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiCommandPtr internal_;

  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Wraps an existing C-style object that is managed by its parent.
     * NOTE: this should not be used except by internal library functions!
     */
    Command(HebiCommandPtr );
    #endif // DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Move constructor (necessary for containment in STL template classes)
     */
    Command(Command&& other);
    /**
     * \brief Cleans up command object as necessary.
     */
    virtual ~Command() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Submessages -------------------------------------------------------------

    /// Any available digital or analog output pins on the device.
    Io& io() { return io_; }
    /// Any available digital or analog output pins on the device.
    const Io& io() const { return io_; }
    /// Module settings that are typically changed at a slower rate.
    Settings& settings() { return settings_; }
    /// Module settings that are typically changed at a slower rate.
    const Settings& settings() const { return settings_; }
    /// Actuator-specific commands.
    Actuator& actuator() { return actuator_; }
    /// Actuator-specific commands.
    const Actuator& actuator() const { return actuator_; }

    // Subfields -------------------------------------------------------------

    #ifndef DOXYGEN_OMIT_INTERNAL
    /// Values for internal debug functions (channel 1-9 available).
    NumberedFloatField& debug();
    /// Values for internal debug functions (channel 1-9 available).
    const NumberedFloatField& debug() const;
    #endif // DOXYGEN_OMIT_INTERNAL
    /// The module's LED.
    LedField& led();
    /// The module's LED.
    const LedField& led() const;

  private:
    Io io_;
    Settings settings_;
    Actuator actuator_;

    NumberedFloatField debug_;
    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Command)

    /* Disable move assigment operator. */
    Command& operator= (const Command&& other) = delete;
};

} // namespace hebi
