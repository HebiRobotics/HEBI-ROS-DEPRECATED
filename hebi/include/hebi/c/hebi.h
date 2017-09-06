#pragma once

// Types
#include "hebi_mac_address.h"
#include "hebi_vector_3_f.h"
// Messages
#include "hebi_command.h"
#include "hebi_feedback.h"
#include "hebi_info.h"
#include "hebi_group_command.h"
#include "hebi_group_feedback.h"
#include "hebi_group_info.h"
// Core
#include "hebi_lookup.h"
#include "hebi_group.h"
// Trajectory API
#include "hebi_trajectory.h"
// Kinematic API
#include "hebi_kinematic_parameters.h"
#include "hebi_kinematics.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * \brief Gets the version for the library
 */
void hebiGetLibraryVersion(int* major, int* minor, int* revision);

/**
 * \brief Frees all resources created by the library.  Note: any calls to the
 * HEBI library functions after this will result in undefined behavior!
 */
void hebiCleanup();

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif
