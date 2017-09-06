#pragma once

#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

#define MAC_BYTES 6
typedef struct _HebiMacAddress {
  uint8_t bytes_[MAC_BYTES];
} HebiMacAddress;
#undef MAC_BYTES

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
