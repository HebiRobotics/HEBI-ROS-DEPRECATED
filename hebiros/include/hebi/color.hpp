#pragma once

#include <cstdint>

namespace hebi {

/// \brief Structure to describe an RGB color.
struct Color {
  public:
    /// \brief Creates a color object with zero for the red, green, blue, and
    /// alpha channels.
    Color() {}


    /// \brief Creates a color from the given red, green, blue, and alpha
    /// channel values.
    ///
    /// Each parameter should be between 0 and 255.
    Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
      : r_(r), g_(g), b_(b), a_(a) {}

    /// \brief Creates a color from the given red, green, and blue values.
    ///
    /// This sets the alpha channel to "255".
    /// Each parameter should be between 0 and 255.
    Color(uint8_t r, uint8_t g, uint8_t b)
      : r_(r), g_(g), b_(b), a_(255) {}

    /// Returns the red channel; value is between 0 and 255.
    uint8_t getRed() const { return r_; }
    /// Returns the green channel; value is between 0 and 255.
    uint8_t getGreen() const { return g_; }
    /// Returns the blue channel; value is between 0 and 255.
    uint8_t getBlue() const { return b_; }
    /// Returns the alpha channel; value is between 0 and 255. '0' indicates
    /// the module has control over this color, and any other value indicates
    /// that there is a command overriding this channel.
    /// For feedback/info values, this value should currently be ignored.
    uint8_t getAlpha() const { return a_; }

    void setRed(uint8_t r) { r_ = r; }
    void setGreen(uint8_t g) { g_ = g; }
    void setBlue(uint8_t b) { b_ = b; }
    void setAlpha(uint8_t a) { a_ = a; }

  private:
    uint8_t r_{};
    uint8_t g_{};
    uint8_t b_{};
    uint8_t a_{};
};

} // namespace hebi
