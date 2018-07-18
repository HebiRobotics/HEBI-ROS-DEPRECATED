#pragma once

#include <iostream>

/**
 * A file which defines basic functionality for blocking keyboard input, without
 * needing to press <enter>.  This isn't necessarily the most optimal solution,
 * but this is a basic starting point for the purpose of input for the examples.
 */

/// Macros needed for getting keyboard input without waiting user needing to
/// press <enter>
#if defined(_WIN32) || defined(_WIN64) // On Windows
    #include <conio.h>
    #define hebi_getchar_init()
    #define hebi_getchar() _getch()
    #define hebi_getchar_cleanup()
#elif defined(__APPLE__) || defined(TARGET_OS_OSX) // On OSX
    #define hebi_getchar_init() system ("/bin/stty raw")
    #define hebi_getchar() getchar()
    #define hebi_getchar_cleanup() system ("/bin/stty -raw")
#else // On Linux
    #include <stdlib.h>
    #define hebi_getchar_init() system ("/bin/stty raw")
    #define hebi_getchar() getchar()
    #define hebi_getchar_cleanup() system ("/bin/stty -raw")
#endif

namespace hebi {
namespace util {

class Input
{
public:
  static int getChar()
  {
    init();
    return getchar();
  }
private:
  static void init()
  {
    // Basically, the Meyer's singleton -- ensures (in a thread safe way) that
    // 'hebi_getchar_init' only ever gets called once.
    static Input keyboard;
  }
  Input()
  {
    hebi_getchar_init();
  }
  ~Input()
  {
    hebi_getchar_cleanup();
  }
};

} // namespace util
} // namespace hebi

