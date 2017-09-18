#pragma once

#include "hebi.h"
#include "Eigen/Eigen"
#include "feedback.hpp"
#include <vector>

namespace hebi {

/**
 * \brief A list of Feedback objects that can be received from a Group of
 * modules; the size() must match the number of modules in the group.
 */
class GroupFeedback final
{
  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style group feedback object.
     * NOTE: this should not be used except by library functions!
     */
    HebiGroupFeedbackPtr internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    const bool manage_pointer_lifetime_;
    /**
     * The number of modules in this group feedback.
     */
    const int number_of_modules_;
    /**
     * The list of Feedback subobjects
     */
    std::vector<Feedback> feedbacks_;

  public:
    /**
     * \brief Create a group feedback with the specified number of modules.
     */
    GroupFeedback(int number_of_modules);
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * Wraps an existing C-style feedback object; object lifetime is assumed to
     * be managed by the caller.
     * NOTE: this should not be used except by internal library functions!
     */
    GroupFeedback(HebiGroupFeedbackPtr group_feedback);
    #endif // DOXYGEN_OMIT_INTERNAL

    /**
     * \brief Destructor cleans up group feedback object as necessary.
     */
    virtual ~GroupFeedback() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * \brief Returns the number of module feedbacks in this group feedback.
     */
    int size() const;

    /**
     * \brief Access the feedback for an individual module.
     */
    const Feedback& operator[](int index) const;

    /**
     * \brief Convenience function for returning feedback board temperature values.
     */
    Eigen::VectorXd getBoardTemperature() const;
    /**
     * \brief Convenience function for returning feedback processor temperature values.
     */
    Eigen::VectorXd getProcessorTemperature() const;
    /**
     * \brief Convenience function for returning feedback voltage values.
     */
    Eigen::VectorXd getVoltage() const;
    /**
     * \brief Convenience function for returning feedback deflection values.
     */
    Eigen::VectorXd getDeflection() const;
    /**
     * \brief Convenience function for returning feedback deflection velocity values.
     */
    Eigen::VectorXd getDeflectionVelocity() const;
    /**
     * \brief Convenience function for returning feedback motor velocity values.
     */
    Eigen::VectorXd getMotorVelocity() const;
    /**
     * \brief Convenience function for returning feedback motor current values.
     */
    Eigen::VectorXd getMotorCurrent() const;
    /**
     * \brief Convenience function for returning feedback motor sensor temperature values.
     */
    Eigen::VectorXd getMotorSensorTemperature() const;
    /**
     * \brief Convenience function for returning feedback motor winding current values.
    */
    Eigen::VectorXd getMotorWindingCurrent() const;
    /**
     * \brief Convenience function for returning feedback motor winding temperature values.
     */
    Eigen::VectorXd getMotorWindingTemperature() const;
    /**
     * \brief Convenience function for returning feedback motor housing temperature values.
     */
    Eigen::VectorXd getMotorHousingTemperature() const;

    /**
     * \brief Convenience function for returning feedback position values.
     */
    Eigen::VectorXd getPosition() const;
    /**
     * \brief Convenience function for returning feedback velocity values.
     */
    Eigen::VectorXd getVelocity() const;
    /**
     * \brief Convenience function for returning feedback effort values.
     */
    Eigen::VectorXd getEffort() const;

    /**
     * \brief Convenience function for returning commanded position values.
     */
    Eigen::VectorXd getPositionCommand() const;
    /**
     * \brief Convenience function for returning commanded velocity values.
     */
    Eigen::VectorXd getVelocityCommand() const;
    /**
     * \brief Convenience function for returning commanded effort values.
     */
    Eigen::VectorXd getEffortCommand() const;

    /**
     * \brief Convenience function for returning feedback accelerometer values.
     */
    Eigen::MatrixX3d getAccelerometer() const;
    /**
     * \brief Convenience function for returning feedback gyroscope values.
     */
    Eigen::MatrixX3d getGyro() const;

    /**
     *\brief Convenience function for returning feedback board temperature values.
     */
    void getBoardTemperature(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback processor temperature values.
     */
    void getProcessorTemperature(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback voltage values.
     */
    void getVoltage(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback deflection values.
     */
    void getDeflection(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback deflection velocity values.
     */
    void getDeflectionVelocity(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback motor velocity values.
     */
    void getMotorVelocity(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback motor current values.
     */
    void getMotorCurrent(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback motor sensor temperature values.
     */
    void getMotorSensorTemperature(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback motor winding current values.
    */
    void getMotorWindingCurrent(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback motor winding temperature values.
     */
    void getMotorWindingTemperature(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback motor housing temperature values.
     */
    void getMotorHousingTemperature(Eigen::VectorXd& out) const;

    /**
     * \brief Convenience function for returning feedback position values.
     */
    void getPosition(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback velocity values.
     */
    void getVelocity(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning feedback effort values.
     */
    void getEffort(Eigen::VectorXd& out) const;

    /**
     * \brief Convenience function for returning commanded position values.
     */
    void getPositionCommand(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning commanded velocity values.
     */
    void getVelocityCommand(Eigen::VectorXd& out) const;
    /**
     * \brief Convenience function for returning commanded effort values.
     */
    void getEffortCommand(Eigen::VectorXd& out) const;

    /**
     * \brief Convenience function for returning feedback accelerometer values.
     */
    void getAccelerometer(Eigen::MatrixX3d& out) const;
    /**
     * \brief Convenience function for returning feedback gyroscope values.
     */
    void getGyro(Eigen::MatrixX3d& out) const;

};

} // namespace hebi
