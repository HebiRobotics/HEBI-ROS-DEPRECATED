#pragma once

#include "hebi.h"
#include "Eigen/Eigen"
#include "info.hpp"
#include <vector>

namespace hebi {

/**
 * \brief A list of Info objects that can be received from a Group of modules;
 * the size() must match the number of modules in the group.
 */
class GroupInfo final
{
  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style group info object.
     * NOTE: this should not be used except by library functions!
     */
    HebiGroupInfoPtr internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    const bool manage_pointer_lifetime_;
    /**
     * The number of modules in this group info.
     */
    const int number_of_modules_;
    /**
     * The list of Info subobjects
     */
    std::vector<Info> infos_;

  public:
    /**
     * \brief Create a group info with the specified number of modules.
     */
    GroupInfo(int number_of_modules);

    /**
     * \brief Destructor cleans up group info object as necessary.
     */
    virtual ~GroupInfo() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * \brief Returns the number of module infos in this group info.
     */
    int size() const;

    /**
     * \brief Access the info for an individual module.
     */
    const Info& operator[](int index) const;
    
    /**
     * \brief Export the gains from this GroupInfo object into a file, creating it as necessary.
     * \param file The filename (or path + filename) to the file to write to.
     */
    bool writeGains(const std::string& file);

    /**
     * \brief Convenience function for returning spring constant values.
     */
    Eigen::VectorXd getSpringConstant() const;

    /**
     * \brief Convenience function for returning spring constant values.
     */
    void getSpringConstant(Eigen::VectorXd& out) const;

};

} // namespace hebi
