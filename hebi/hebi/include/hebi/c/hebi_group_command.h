/* This file has been automatically generated. Do not edit by hand. */

#pragma once

#include "hebi_command.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * @brief The C-style's API representation of a command object for a
 * group of modules.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 */
typedef struct _HebiGroupCommand* HebiGroupCommandPtr;

/**
 * @brief Creates a GroupCommand for a group with the specified number of
 * modules.
 *
 * @param size The number of modules in the group.
 *
 * @returns A pointer to a new GroupCommand object. This must be released
 * with @c hebiGroupCommandRelease(HebiGroupCommandPtr).
 */
HebiGroupCommandPtr hebiGroupCommandCreate(int size);
/**
 * @brief Return the number of modules in this group Command.
 *
 * @returns The number of module commands in this group command.
 */
int hebiGroupCommandGetSize(HebiGroupCommandPtr);
/**
 * @brief Import gains from a file into a GroupCommand object.
 */
HebiStatusCode hebiGroupCommandReadGains(HebiGroupCommandPtr, const char* file);
/**
 * @brief Export gains from a GroupCommand object into a file.
 */
HebiStatusCode hebiGroupCommandWriteGains(HebiGroupCommandPtr, const char* file);
/**
 * @brief Get an individual command for a particular module at index
 * @c module_index.
 *
 * @param module_index The index to retrieve the module command; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior. 
 *
 * @returns The command corresponding to the module at index
 * @c module_index.
 */
HebiCommandPtr hebiGroupCommandGetModuleCommand(HebiGroupCommandPtr, int module_index);
/**
 * \brief Frees resources created by the GroupCommand object.
 *
 * The GroupCommandPtr should no longer be used after this function is
 * called!
 */
void hebiGroupCommandRelease(HebiGroupCommandPtr);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
