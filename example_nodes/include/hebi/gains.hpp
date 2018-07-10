#pragma once

namespace hebi {

template<typename MessageType,
         typename FloatFieldType, typename BoolFieldType,
         typename FloatEnumType, typename BoolEnumType>
class Gains final
{
  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    Gains(MessageType internal, FloatEnumType kp_gain, BoolEnumType d_on_error_gain)
      : internal_(internal),
        kp_(internal, kp_gain),
        ki_(internal, static_cast<FloatEnumType>(kp_gain + 1)),
        kd_(internal, static_cast<FloatEnumType>(kp_gain + 2)),
        feed_forward_(internal, static_cast<FloatEnumType>(kp_gain + 3)),
        dead_zone_(internal, static_cast<FloatEnumType>(kp_gain + 4)),
        i_clamp_(internal, static_cast<FloatEnumType>(kp_gain + 5)),
        punch_(internal, static_cast<FloatEnumType>(kp_gain + 6)),
        min_target_(internal, static_cast<FloatEnumType>(kp_gain + 7)),
        max_target_(internal, static_cast<FloatEnumType>(kp_gain + 8)),
        target_lowpass_(internal, static_cast<FloatEnumType>(kp_gain + 9)),
        min_output_(internal, static_cast<FloatEnumType>(kp_gain + 10)),
        max_output_(internal, static_cast<FloatEnumType>(kp_gain + 11)),
        output_lowpass_(internal, static_cast<FloatEnumType>(kp_gain + 12)),
        d_on_error_(internal, d_on_error_gain)
    {
    }
    #endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.
            
    // Subfields ----------------

    /// Proportional PID gain
    FloatFieldType& kP() { return kp_; }
    /// Proportional PID gain
    const FloatFieldType& kP() const { return kp_; }
    /// Integral PID gain
    FloatFieldType& kI() { return ki_; }
    /// Integral PID gain
    const FloatFieldType& kI() const { return ki_; }
    /// Derivative PID gain
    FloatFieldType& kD() { return kd_; }
    /// Derivative PID gain
    const FloatFieldType& kD() const { return kd_; }
    /// Feed forward term (this term is multiplied by the target and added to the output).
    FloatFieldType& feedForward() { return feed_forward_; }
    /// Feed forward term (this term is multiplied by the target and added to the output).
    const FloatFieldType& feedForward() const { return feed_forward_; }
    /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
    FloatFieldType& deadZone() { return dead_zone_; }
    /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
    const FloatFieldType& deadZone() const { return dead_zone_; }
    /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
    FloatFieldType& iClamp() { return i_clamp_; }
    /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
    const FloatFieldType& iClamp() const { return i_clamp_; }
    /// Constant offset to the PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
    FloatFieldType& punch() { return punch_; }
    /// Constant offset to the PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
    const FloatFieldType& punch() const { return punch_; }
    /// Minimum allowed value for input to the PID controller
    FloatFieldType& minTarget() { return min_target_; }
    /// Minimum allowed value for input to the PID controller
    const FloatFieldType& minTarget() const { return min_target_; }
    /// Maximum allowed value for input to the PID controller
    FloatFieldType& maxTarget() { return max_target_; }
    /// Maximum allowed value for input to the PID controller
    const FloatFieldType& maxTarget() const { return max_target_; }
    /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
    FloatFieldType& targetLowpass() { return target_lowpass_; }
    /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
    const FloatFieldType& targetLowpass() const { return target_lowpass_; }
    /// Output from the PID controller is limited to a minimum of this value.
    FloatFieldType& minOutput() { return min_output_; }
    /// Output from the PID controller is limited to a minimum of this value.
    const FloatFieldType& minOutput() const { return min_output_; }
    /// Output from the PID controller is limited to a maximum of this value.
    FloatFieldType& maxOutput() { return max_output_; }
    /// Output from the PID controller is limited to a maximum of this value.
    const FloatFieldType& maxOutput() const { return max_output_; }
    /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
    FloatFieldType& outputLowpass() { return output_lowpass_; }
    /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
    const FloatFieldType& outputLowpass() const { return output_lowpass_; }
    /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
    BoolFieldType& dOnError() { return d_on_error_; }
    /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
    const BoolFieldType& dOnError() const { return d_on_error_; }
            
  private:
    MessageType const internal_;

    FloatFieldType kp_;
    FloatFieldType ki_;
    FloatFieldType kd_;
    FloatFieldType feed_forward_;
    FloatFieldType dead_zone_;
    FloatFieldType i_clamp_;
    FloatFieldType punch_;
    FloatFieldType min_target_;
    FloatFieldType max_target_;
    FloatFieldType target_lowpass_;
    FloatFieldType min_output_;
    FloatFieldType max_output_;
    FloatFieldType output_lowpass_;
    BoolFieldType d_on_error_;

    HEBI_DISABLE_COPY_MOVE(Gains)
};

} // namespace hebi
