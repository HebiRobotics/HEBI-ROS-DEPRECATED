/* This file has been automatically generated. Do not edit by hand. */

#pragma once

#include "hebi_feedback.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * @brief The C-style's API representation of a feedback object for a
 * group of modules.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 */
typedef struct _HebiGroupFeedback* HebiGroupFeedbackPtr;

/**
 * @brief Creates a GroupFeedback for a group with the specified number of
 * modules.
 *
 * @param size The number of modules in the group.
 *
 * @returns A pointer to a new GroupFeedback object. This must be released
 * with @c hebiGroupFeedbackRelease(HebiGroupFeedbackPtr).
 */
HebiGroupFeedbackPtr hebiGroupFeedbackCreate(int size);
/**
 * @brief Return the number of modules in this group Feedback.
 *
 * @returns The number of module feedbacks in this group feedback.
 */
int hebiGroupFeedbackGetSize(HebiGroupFeedbackPtr);
/**
 * @brief Get an individual feedback for a particular module at index
 * @c module_index.
 *
 * @param module_index The index to retrieve the module feedback; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior. 
 *
 * @returns The feedback corresponding to the module at index
 * @c module_index.
 */
HebiFeedbackPtr hebiGroupFeedbackGetModuleFeedback(HebiGroupFeedbackPtr, int module_index);
/**
 * \brief Frees resources created by the GroupFeedback object.
 *
 * The GroupFeedbackPtr should no longer be used after this function is
 * called!
 */
void hebiGroupFeedbackRelease(HebiGroupFeedbackPtr);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
