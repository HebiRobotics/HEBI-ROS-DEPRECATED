#pragma once

#include <cstdint>

namespace hebi {

/// \brief Structure to describe an RGB color.
struct Color {
  public:
    /// \brief Creates a color from the given red, green, and blue channel values.
    ///
    /// Each parameter should be between 0 and 255.
    Color(uint8_t r, uint8_t g, uint8_t b) : r_(r), g_(g), b_(b) {}

    /// Returns the red channel; value is between 0 and 255.
    uint8_t getRed() const { return r_; }
    /// Returns the green channel; value is between 0 and 255.
    uint8_t getGreen() const { return g_; }
    /// Returns the blue channel; value is between 0 and 255.
    uint8_t getBlue() const { return b_; }

  private:
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;

};

#ifndef DOXYGEN_OMIT_INTERNAL
/// \brief Typedef of Color structure for improved syntax.
typedef Color Color;
#endif // DOXYGEN_OMIT_INTERNAL

} // namespace hebi
