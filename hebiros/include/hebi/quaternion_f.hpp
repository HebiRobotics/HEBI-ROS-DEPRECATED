#pragma once

#include "hebi.h"

namespace hebi {

/// \brief Structure to hold a floating point quaternion (i.e., w/x/y/z components)
struct Quaternionf {
  public:
    /// \brief Create a floating point quaternion from three floating point
    /// components.
    Quaternionf(float w, float x, float y, float z) : w_(w), x_(x), y_(y), z_(z) {}

    #ifndef DOXYGEN_OMIT_INTERNAL
    /// \brief Method to create a C++ scoped hebi::Quaternionf object from an
    /// internal HEBI C API pointer.  Internal use only.
    Quaternionf(const HebiQuaternionf& src) : w_(src.w), x_(src.x), y_(src.y), z_(src.z) {}
    #endif // DOXYGEN_OMIT_INTERNAL

    /// Returns the W component of the quaternion.
    float getW() const { return w_; }
    /// Returns the X component of the quaternion.
    float getX() const { return x_; }
    /// Returns the Y component of the quaternion.
    float getY() const { return y_; }
    /// Returns the Z component of the quaternion.
    float getZ() const { return z_; }

  private:
    float w_;
    float x_;
    float y_;
    float z_;
};

} // namespace hebi
