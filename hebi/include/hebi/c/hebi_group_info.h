/* This file has been automatically generated. Do not edit by hand. */

#pragma once

#include "hebi_info.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * @brief The C-style's API representation of a info object for a
 * group of modules.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 */
typedef struct _HebiGroupInfo* HebiGroupInfoPtr;

/**
 * @brief Creates a GroupInfo for a group with the specified number of
 * modules.
 *
 * @param size The number of modules in the group.
 *
 * @returns A pointer to a new GroupInfo object. This must be released
 * with @c hebiGroupInfoRelease(HebiGroupInfoPtr).
 */
HebiGroupInfoPtr hebiGroupInfoCreate(int size);
/**
 * @brief Return the number of modules in this group Info.
 *
 * @returns The number of module infos in this group info.
 */
int hebiGroupInfoGetSize(HebiGroupInfoPtr);
/**
 * @brief Export gains from a GroupInfo object into a file.
 */
HebiStatusCode hebiGroupInfoWriteGains(HebiGroupInfoPtr, const char* file);
/**
 * @brief Get an individual info for a particular module at index
 * @c module_index.
 *
 * @param module_index The index to retrieve the module info; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior. 
 *
 * @returns The info corresponding to the module at index
 * @c module_index.
 */
HebiInfoPtr hebiGroupInfoGetModuleInfo(HebiGroupInfoPtr, int module_index);
/**
 * \brief Frees resources created by the GroupInfo object.
 *
 * The GroupInfoPtr should no longer be used after this function is
 * called!
 */
void hebiGroupInfoRelease(HebiGroupInfoPtr);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
