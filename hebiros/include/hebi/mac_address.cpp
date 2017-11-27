#include "mac_address.hpp"

namespace hebi {

uint8_t MacAddress::hexToInt(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return 0;
}

uint8_t MacAddress::byteFromHexPair(char c1, char c2)
{
  return hexToInt(c1) * 16 + hexToInt(c2);
}

MacAddress::MacAddress()
{
  internal_.bytes_[0] = 0;
  internal_.bytes_[1] = 0;
  internal_.bytes_[2] = 0;
  internal_.bytes_[3] = 0;
  internal_.bytes_[4] = 0;
  internal_.bytes_[5] = 0;
}

MacAddress MacAddress::fromBytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f)
{
  MacAddress addr;
  addr.internal_.bytes_[0] = a;
  addr.internal_.bytes_[1] = b;
  addr.internal_.bytes_[2] = c;
  addr.internal_.bytes_[3] = d;
  addr.internal_.bytes_[4] = e;
  addr.internal_.bytes_[5] = f;
  return addr;
}

bool MacAddress::setToHexString(std::string mac_str)
{
  if (!isHexStringValid(mac_str))
    return false;
  internal_.bytes_[0] = byteFromHexPair(mac_str[0], mac_str[1]);
  internal_.bytes_[1] = byteFromHexPair(mac_str[3], mac_str[4]);
  internal_.bytes_[2] = byteFromHexPair(mac_str[6], mac_str[7]);
  internal_.bytes_[3] = byteFromHexPair(mac_str[9], mac_str[10]);
  internal_.bytes_[4] = byteFromHexPair(mac_str[12], mac_str[13]);
  internal_.bytes_[5] = byteFromHexPair(mac_str[15], mac_str[16]);
  return true;
}

uint8_t MacAddress::operator[](std::size_t idx)
{
  return internal_.bytes_[idx];
}

bool MacAddress::isHexDigitValid(char c)
{
  return ((c >= '0' && c <= '9') ||
          (c >= 'a' && c <= 'f') ||
          (c >= 'A' && c <= 'F'));
}


bool MacAddress::isHexStringValid(std::string mac_str)
{
  int len = mac_str.size();
  if (len != 17) // 6 * 2 (digits) + 5 (":"s)
    return false;
  for (int j = 2; j < len; j += 3)
    if (mac_str[j] != ':')
      return false;
  for (int j = 0; j < len; j += 3)
    if (!isHexDigitValid(mac_str[j]) || !isHexDigitValid(mac_str[j+1]))
      return false;
  return true;
}

} // namespace hebi
