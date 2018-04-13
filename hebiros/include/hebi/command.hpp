#pragma once

#include "hebi.h"
#include "color.hpp"
#include <string>
#include "util.hpp"
#include "gains.hpp"

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
    enum class ControlStrategy {
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
        bool has(size_t fieldNumber) const;
        /// \brief If the particular numbered subvalue of this field has a
        /// value, returns that value; otherwise returns a default.
        ///
        /// \param fieldNumber Which subvalue to get; valid values for
        /// fieldNumber depend on the field type.
        float get(size_t fieldNumber) const;
        /// \brief Sets the particular numbered subvalue of this field to a
        /// given value.
        ///
        /// \param fieldNumber Which subvalue to set; valid values for
        /// fieldNumber depend on the field type.
        void set(size_t fieldNumber, float value);
        /// \brief Removes any currently set value for the numbered subvalue of
        /// this field.
        ///
        /// \param fieldNumber Which subvalue to clear; valid values for
        /// fieldNumber depend on the field type.
        void clear(size_t fieldNumber);

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
    template <typename T>
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
        T get() const { int32_t ret{}; hebiCommandGetEnum(internal_, field_, &ret); return static_cast<T>(ret); }
        /// \brief Sets the field to a given value.
        void set(T _value) { int32_t value = static_cast<int32_t>(_value); hebiCommandSetEnum(internal_, field_, &value); }
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
        bool hasInt(size_t pinNumber) const;
        /// \brief True if (and only if) the particular numbered pin in this
        /// bank has an floating point (e.g., analog or PWM) value.
        ///
        /// \param pinNumber Which pin to check; valid values for pinNumber
        /// depend on the bank.
        bool hasFloat(size_t pinNumber) const;
        /// \brief If this numbered pin in this bank has an integer (e.g.,
        /// digital) value, returns that value; otherwise returns a default.
        ///
        /// \param pinNumber Which pin to get; valid values for pinNumber
        /// depend on the bank.
        int64_t getInt(size_t pinNumber) const;
        /// \brief If this numbered pin in this bank has an floating point
        /// (e.g., analog or PWM) value, returns that value; otherwise returns a
        /// default.
        ///
        /// \param pinNumber Which pin to get; valid values for pinNumber
        /// depend on the bank.
        float getFloat(size_t pinNumber) const;
        /// \brief Sets the particular pin to a integer value (representing a
        /// digital output).
        ///
        /// \param pinNumber Which pin to set; valid values for pinNumber
        /// depend on the bank.
        void setInt(size_t pinNumber, int64_t value);
        /// \brief Sets the particular pin to a floating point value
        /// (representing a PWM output).
        ///
        /// \param pinNumber Which pin to set; valid values for pinNumber
        /// depend on the bank.
        void setFloat(size_t pinNumber, float value);
        /// \brief Removes any currently set value for this pin.
        ///
        /// \param pinNumber Which pin to clear; valid values for pinNumber
        /// depend on the bank.
        void clear(size_t pinNumber);

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
        /// \brief Returns true if the LED command has been set, and false
        /// otherwise.
        ///
        /// A command is "set" if there is an override color specified _or_ if
        /// the module is being commanded to resume control of the LED.
        /// If this returns @c false , it indicates that the current state of
        /// the LED will be maintained.
        bool has() const;
        /// \brief Returns the current LED command.
        ///
        /// If the alpha channel is '0', this command indicates that the module
        /// should resume control of the LED (and the R, G, and B values are
        /// ignored).
        /// If the alpha channel is '1', the R, G, and B values in this command
        /// will override the module's control of the LED.
        Color get() const;
        /// \brief Commands a color that overrides the module's control of the
        /// LED (if the alpha channel is 255), or specifies the the module
        /// should resume control of the LED color (if the alpha channel is 0).
        /// Values of the alpha channel from 1 to 254 are reserved for future
        /// use.
        void set(const Color& color);
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

    using CommandGains = Gains<HebiCommandPtr, FloatField, BoolField, HebiCommandFloatField, HebiCommandBoolField>;

    /// Module settings that are typically changed at a slower rate.
    class Settings final
    {
      // Note: this is 'protected' instead of 'private' for easier use with Doxygen
      protected:
        /// Actuator-specific settings, such as controller gains.
        class Actuator final
        {
          public:
            #ifndef DOXYGEN_OMIT_INTERNAL
            Actuator(HebiCommandPtr internal)
              : internal_(internal),
                position_gains_(internal, HebiCommandFloatPositionKp, HebiCommandBoolPositionDOnError),
                velocity_gains_(internal, HebiCommandFloatVelocityKp, HebiCommandBoolVelocityDOnError),
                effort_gains_(internal, HebiCommandFloatEffortKp, HebiCommandBoolEffortDOnError),
                spring_constant_(internal, HebiCommandFloatSpringConstant),
                reference_position_(internal, HebiCommandFloatReferencePosition),
                reference_effort_(internal, HebiCommandFloatReferenceEffort),
                position_limit_min_(internal, HebiCommandHighResAnglePositionLimitMin),
                position_limit_max_(internal, HebiCommandHighResAnglePositionLimitMax),
                control_strategy_(internal, HebiCommandEnumControlStrategy)
            {
            }
            #endif // DOXYGEN_OMIT_INTERNAL
        
            // With all submessage and field getters: Note that the returned reference
            // should not be used after the lifetime of this parent.
        
            // Submessages ----------------
        
            /// Controller gains for the position PID loop.
            CommandGains& positionGains() { return position_gains_; }
            /// Controller gains for the position PID loop.
            const CommandGains& positionGains() const { return position_gains_; }
            /// Controller gains for the velocity PID loop.
            CommandGains& velocityGains() { return velocity_gains_; }
            /// Controller gains for the velocity PID loop.
            const CommandGains& velocityGains() const { return velocity_gains_; }
            /// Controller gains for the effort PID loop.
            CommandGains& effortGains() { return effort_gains_; }
            /// Controller gains for the effort PID loop.
            const CommandGains& effortGains() const { return effort_gains_; }
        
            // Subfields ----------------
        
            /// The spring constant of the module.
            FloatField& springConstant() { return spring_constant_; }
            /// The spring constant of the module.
            const FloatField& springConstant() const { return spring_constant_; }
            /// The internal encoder reference offset (setting this matches the current position to the given reference command)
            FloatField& referencePosition() { return reference_position_; }
            /// The internal encoder reference offset (setting this matches the current position to the given reference command)
            const FloatField& referencePosition() const { return reference_position_; }
            /// The internal effort reference offset (setting this matches the current effort to the given reference command)
            FloatField& referenceEffort() { return reference_effort_; }
            /// The internal effort reference offset (setting this matches the current effort to the given reference command)
            const FloatField& referenceEffort() const { return reference_effort_; }
            /// The firmware safety limit for the minimum allowed position.
            HighResAngleField& positionLimitMin() { return position_limit_min_; }
            /// The firmware safety limit for the minimum allowed position.
            const HighResAngleField& positionLimitMin() const { return position_limit_min_; }
            /// The firmware safety limit for the maximum allowed position.
            HighResAngleField& positionLimitMax() { return position_limit_max_; }
            /// The firmware safety limit for the maximum allowed position.
            const HighResAngleField& positionLimitMax() const { return position_limit_max_; }
            /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
            EnumField<ControlStrategy>& controlStrategy() { return control_strategy_; }
            /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
            const EnumField<ControlStrategy>& controlStrategy() const { return control_strategy_; }
        
          private:
            HebiCommandPtr const internal_;
        
            CommandGains position_gains_;
            CommandGains velocity_gains_;
            CommandGains effort_gains_;
        
            FloatField spring_constant_;
            FloatField reference_position_;
            FloatField reference_effort_;
            HighResAngleField position_limit_min_;
            HighResAngleField position_limit_max_;
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
    ~Command() noexcept; /* annotating specified destructor as noexcept is best-practice */

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
    /// Restart the module.
    FlagField& reset();
    /// Restart the module.
    const FlagField& reset() const;
    /// Boot the module from bootloader into application.
    FlagField& boot();
    /// Boot the module from bootloader into application.
    const FlagField& boot() const;
    /// Stop the module from automatically booting into application.
    FlagField& stopBoot();
    /// Stop the module from automatically booting into application.
    const FlagField& stopBoot() const;
    /// The module's LED.
    LedField& led();
    /// The module's LED.
    const LedField& led() const;

  private:
    Io io_;
    Settings settings_;
    Actuator actuator_;

    NumberedFloatField debug_;
    FlagField reset_;
    FlagField boot_;
    FlagField stop_boot_;
    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Command)

    /* Disable move assigment operator. */
    Command& operator= (const Command&& other) = delete;
};

} // namespace hebi
