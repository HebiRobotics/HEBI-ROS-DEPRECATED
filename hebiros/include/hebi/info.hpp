#pragma once

#include "hebi.h"
#include "color.hpp"
#include <string>
#include "util.hpp"
#include "gains.hpp"

namespace hebi {

/// \brief Info objects have various fields representing the module state; which
/// fields are populated depends on the module type and various other settings.
/// 
/// This object has a hierarchical structure -- there are some direct general-purpose
/// fields at the top level, and many more specific fields contained in different
/// nested subobjects.
/// 
/// The subobjects contain references to the parent info object, and so should not be
/// used after the parent object has been destroyed.
/// 
/// The fields in the info object are typed; generally, these are optional-style
/// read-only fields (i.e., have the concept of has/get), although the return types
/// and exact interface vary slightly between fields. Where appropriate, the explicit
/// bool operator has been overridden so that you can shortcut @c if(field.has()) by
/// calling @c if(field).
/// 
/// Although this header file can be used to look at the hierarchy of the messages,
/// in general the online documentation at apidocs.hebi.us presents this information.
/// in a more readable form.
class Info final
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

    enum class CalibrationState {
      /// The module has been calibrated; this is the normal state
      Normal,
      /// The current has not been calibrated
      UncalibratedCurrent,
      /// The factory zero position has not been set
      UncalibratedPosition,
      /// The effort (e.g., spring nonlinearity) has not been calibrated
      UncalibratedEffort,
    };

  // Note: this is 'protected' instead of 'private' for easier use with Doxygen
  protected:
    /// \brief A message field representable by a single-precision floating point value.
    class FloatField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FloatField(HebiInfoPtr internal, HebiInfoFloatField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::FloatField& f = parent.myField();
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

      private:
        HebiInfoPtr const internal_;
        HebiInfoFloatField const field_;

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
        HighResAngleField(HebiInfoPtr internal, HebiInfoHighResAngleField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::HighResAngleField& f = parent.myField();
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

      private:
        HebiInfoPtr const internal_;
        HebiInfoHighResAngleField const field_;

        HEBI_DISABLE_COPY_MOVE(HighResAngleField)
    };

    /// \brief A message field representable by a bool value.
    class BoolField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        BoolField(HebiInfoPtr internal, HebiInfoBoolField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns false.
        bool get() const;

      private:
        HebiInfoPtr const internal_;
        HebiInfoBoolField const field_;

        HEBI_DISABLE_COPY_MOVE(BoolField)
    };

    /// \brief A message field representable by a std::string.
    class StringField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        StringField(HebiInfoPtr internal, HebiInfoStringField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::StringField& f = parent.myField();
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

      private:
        HebiInfoPtr const internal_;
        HebiInfoStringField const field_;

        HEBI_DISABLE_COPY_MOVE(StringField)
    };

    /// \brief A two-state message field (either set/true or cleared/false).
    class FlagField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FlagField(HebiInfoPtr internal, HebiInfoFlagField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the flag is set without
        /// directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::FlagField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief Returns @c true if the flag is set, false if it is cleared.
        bool has() const;

      private:
        HebiInfoPtr const internal_;
        HebiInfoFlagField const field_;

        HEBI_DISABLE_COPY_MOVE(FlagField)
    };

    /// \brief A message field representable by an enum of a given type.
    template <typename T>
    class EnumField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        EnumField(HebiInfoPtr internal, HebiInfoEnumField field)
          : internal_(internal), field_(field) {}
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::EnumField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const { return has(); }
        /// \brief True if (and only if) the field has a value.
        bool has() const { return (hebiInfoGetEnum(internal_, field_, nullptr) == HebiStatusSuccess); }
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns a default.
        T get() const { int32_t ret{}; hebiInfoGetEnum(internal_, field_, &ret); return static_cast<T>(ret); }

      private:
        HebiInfoPtr const internal_;
        HebiInfoEnumField const field_;

        HEBI_DISABLE_COPY_MOVE(EnumField)
    };

    /// \brief A message field for interfacing with an LED.
    class LedField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        LedField(HebiInfoPtr internal, HebiInfoLedField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the LED color is set
        /// without directly calling @c hasColor().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::LedField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has color!" << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const { return hasColor(); }
        /// \brief Returns true if the LED color is set, and false otherwise.
        bool hasColor() const;
        /// \brief Returns the led color.
        Color getColor() const;

      private:
        HebiInfoPtr const internal_;
        HebiInfoLedField const field_;

        HEBI_DISABLE_COPY_MOVE(LedField)
    };

    using InfoGains = Gains<HebiInfoPtr, FloatField, BoolField, HebiInfoFloatField, HebiInfoBoolField>;

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
            Actuator(HebiInfoPtr internal)
              : internal_(internal),
                position_gains_(internal, HebiInfoFloatPositionKp, HebiInfoBoolPositionDOnError),
                velocity_gains_(internal, HebiInfoFloatVelocityKp, HebiInfoBoolVelocityDOnError),
                effort_gains_(internal, HebiInfoFloatEffortKp, HebiInfoBoolEffortDOnError),
                spring_constant_(internal, HebiInfoFloatSpringConstant),
                position_limit_min_(internal, HebiInfoHighResAnglePositionLimitMin),
                position_limit_max_(internal, HebiInfoHighResAnglePositionLimitMax),
                control_strategy_(internal, HebiInfoEnumControlStrategy)
            {
            }
            #endif // DOXYGEN_OMIT_INTERNAL
        
            // With all submessage and field getters: Note that the returned reference
            // should not be used after the lifetime of this parent.
        
            // Submessages ----------------
        
            /// Controller gains for the position PID loop.
            const InfoGains& positionGains() const { return position_gains_; }
            /// Controller gains for the velocity PID loop.
            const InfoGains& velocityGains() const { return velocity_gains_; }
            /// Controller gains for the effort PID loop.
            const InfoGains& effortGains() const { return effort_gains_; }
        
            // Subfields ----------------
        
            /// The spring constant of the module.
            const FloatField& springConstant() const { return spring_constant_; }
            /// The firmware safety limit for the minimum allowed position.
            const HighResAngleField& positionLimitMin() const { return position_limit_min_; }
            /// The firmware safety limit for the maximum allowed position.
            const HighResAngleField& positionLimitMax() const { return position_limit_max_; }
            /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
            const EnumField<ControlStrategy>& controlStrategy() const { return control_strategy_; }
        
          private:
            HebiInfoPtr const internal_;
        
            InfoGains position_gains_;
            InfoGains velocity_gains_;
            InfoGains effort_gains_;
        
            FloatField spring_constant_;
            HighResAngleField position_limit_min_;
            HighResAngleField position_limit_max_;
            EnumField<ControlStrategy> control_strategy_;
        
            HEBI_DISABLE_COPY_MOVE(Actuator)
        };
    
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Settings(HebiInfoPtr internal)
          : internal_(internal),
            actuator_(internal),
            name_(internal, HebiInfoStringName),
            family_(internal, HebiInfoStringFamily),
            save_current_settings_(internal, HebiInfoFlagSaveCurrentSettings)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Submessages ----------------
    
        /// Actuator-specific settings, such as controller gains.
        const Actuator& actuator() const { return actuator_; }
    
        // Subfields ----------------
    
        /// Gets the name for this module.
        const StringField& name() const { return name_; }
        /// Gets the family for this module.
        const StringField& family() const { return family_; }
        /// Indicates if the module should save the current values of all of its settings.
        const FlagField& saveCurrentSettings() const { return save_current_settings_; }
    
      private:
        HebiInfoPtr const internal_;
    
        Actuator actuator_;
    
        StringField name_;
        StringField family_;
        FlagField save_current_settings_;
    
        HEBI_DISABLE_COPY_MOVE(Settings)
    };

    /// Actuator-specific information.
    class Actuator final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Actuator(HebiInfoPtr internal)
          : internal_(internal),
            calibration_state_(internal, HebiInfoEnumCalibrationState)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Subfields ----------------
    
        /// The calibration state of the module
        const EnumField<CalibrationState>& calibrationState() const { return calibration_state_; }
    
      private:
        HebiInfoPtr const internal_;
    
        EnumField<CalibrationState> calibration_state_;
    
        HEBI_DISABLE_COPY_MOVE(Actuator)
    };

  private:
    /**
     * C-style object; managed by parent.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiInfoPtr internal_;

  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Wraps an existing C-style object that is managed by its parent.
     * NOTE: this should not be used except by internal library functions!
     */
    Info(HebiInfoPtr );
    #endif // DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Move constructor (necessary for containment in STL template classes)
     */
    Info(Info&& other);
    /**
     * \brief Cleans up info object as necessary.
     */
    ~Info() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Submessages -------------------------------------------------------------

    /// Module settings that are typically changed at a slower rate.
    const Settings& settings() const { return settings_; }
    /// Actuator-specific information.
    const Actuator& actuator() const { return actuator_; }

    // Subfields -------------------------------------------------------------

    /// The module's LED.
    const LedField& led() const;

    /// Gets the serial number for this module (e.g., X5-0001).
    const StringField& serial() const { return serial_; }
    
  private:
    Settings settings_;
    Actuator actuator_;

    LedField led_;
  
    StringField serial_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Info)

    /* Disable move assigment operator. */
    Info& operator= (const Info&& other) = delete;
};

} // namespace hebi
