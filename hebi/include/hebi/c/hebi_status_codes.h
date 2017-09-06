#pragma once

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef enum HebiStatusCode {
  HebiStatusSuccess = 0,
  HebiStatusInvalidArgument = 1,
  HebiStatusBufferTooSmall = 2,
  HebiStatusValueNotSet = 3,
  HebiStatusFailure = 4,
  HebiStatusArgumentOutOfRange = 5
} HebiStatusCode;

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
