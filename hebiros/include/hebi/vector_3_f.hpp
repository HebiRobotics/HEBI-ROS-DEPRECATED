#pragma once

#include "hebi.h"

namespace hebi {

/// \brief Structure to hold a 3-D floating point vector (i.e., x/y/z components)
struct Vector3f {
  public:
    /// \brief Create a 3-D floating point vector from three floating point
    /// components.
    Vector3f(float x, float y, float z) : x_(x), y_(y), z_(z) {}

    #ifndef DOXYGEN_OMIT_INTERNAL
    /// \brief Method to create a C++ scoped hebi::Vector3f object from an
    /// internal HEBI C API pointer.  Internal use only.
    Vector3f(const HebiVector3f& src) : x_(src.x), y_(src.y), z_(src.z) {}
    #endif // DOXYGEN_OMIT_INTERNAL

    /// Returns the X component of the vector.
    float getX() const { return x_; }
    /// Returns the Y component of the vector.
    float getY() const { return y_; }
    /// Returns the Z component of the vector.
    float getZ() const { return z_; }

  private:
    float x_;
    float y_;
    float z_;
};

#ifndef DOXYGEN_OMIT_INTERNAL
/// \brief Typedef of Vector3f structure for improved syntax.
typedef Vector3f Vector3f;
#endif // DOXYGEN_OMIT_INTERNAL

} // namespace hebi
