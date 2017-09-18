#pragma once

#include "hebi.h"
#include <iostream>

namespace hebi {

/**
 * \brief A simple wrapper class for internal C-API HebiMacAddress objects
 * to allow interfacing with API calls that use MAC addresses.
 */
class MacAddress final
{
  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style mac address object; this is a plain struct, so it is OK to rely
     * on default copy/move constructors.
     * NOTE: this should not be used except by library functions!
     */
    HebiMacAddress internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * Converts a hex digit to a byte (highest value possible is 15). Assumes
     * valid digit.
     */
    static uint8_t hexToInt(char c);

    /**
     * Converts a hex pair to a byte (0-255 possible). Assumes valid digits.
     */
    static uint8_t byteFromHexPair(char c1, char c2);

  public:
    /**
     * \brief Creates MAC address 00:00:00:00:00:00.
     */
    MacAddress();

    /**
     * \brief Creates a MacAddress from individual bytes
     */
    static MacAddress fromBytes(
      uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f);

    /**
     * \brief Sets the value of the current MacAddress to the value given in
     * 'mac_str'.
     *
     * This value must be a valid string of format dd:dd:dd:dd:dd:dd, where 'd'
     * is a hex digit (0-F, uppercase or lowercase).
     *
     * \returns 'true' on success (valid mac_str), 'false' on failure.
     */
    bool setToHexString(std::string mac_str);

    uint8_t operator[](std::size_t idx);

  private:
    /*
     * Is 'c' a valid hex digit (0-F, upper or lower case)?
     *
     * \returns 'true' if yes, 'false' if no.
     */
    static bool isHexDigitValid(char c);

  public:
    /**
     * \brief Is mac_str a valid string of format dd:dd:dd:dd:dd:dd, where 'd' is a hex
     * digit 0-F.  Lowercase values accepted. 1 if yes, 0 if no.
     *
     * \returns 'true' if mac_str is valid, 'false' otherwise.
     */
    static bool isHexStringValid(std::string mac_str);
};

} // namespace hebi
