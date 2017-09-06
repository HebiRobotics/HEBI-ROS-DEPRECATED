#pragma once

#include "hebi_status_codes.h"
#include "hebi_mac_address.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * \example lookup_example.c
 * How to lookup a group - simple.
 * \example lookup_general_example.c
 * How to lookup a group - generalized using command line arguments.
 * \example lookup_helpers.c
 * A function -- getGroupFromArgs -- to assist with lookups form command line
 * arguments.
 */

/**
 * Maintains a registry of network-connected modules and returns Group objects
 * to the user. Only one Lookup object is needed per application.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 29 Oct 2014
 */
typedef struct _HebiLookup *HebiLookupPtr;
/**
 * A list of entries that represent a snapshot of the state of the lookup object
 * at some point in time.  These entries include network HEBI devices such
 * as actuators.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 10 Feb 2017
 */
typedef struct _HebiLookupEntryList *HebiLookupEntryListPtr;

/**
 * \brief Create a Lookup instance.
 * 
 * Lookup created by this function must be released with 'hebiLookupRelease'
 * when no longer needed.
 *
 * Note that this call invokes a background thread to query the network for
 * modules at regular intervals.
 */
HebiLookupPtr hebiLookupCreate();
/**
 * \brief Frees resources created by the lookup object.
 *
 * Lookup object should no longer be used after this function is called!
 * Note that background query thread is stopped by this function.
 */
void hebiLookupRelease(HebiLookupPtr lookup);

/**
 * \brief Return a snapshot of the contents of the module registry -- i.e.,
 * which modules have been found by the lookup.
 *
 * @param lookup A valid HebiLookup object.
 */
HebiLookupEntryListPtr hebiCreateLookupEntryList(HebiLookupPtr lookup);
/**
 * Gets the number of entries in the lookup entry list.
 *
 * @param lookup_list A valid HebiLookupEntryList object.
 */
int hebiLookupEntryListGetSize(HebiLookupEntryListPtr lookup_list);
/**
 * Gets the name of the given entry in the lookup entry list. Must be a valid
 * index.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * Note - assumes ASCII string encoding.
 *
 * @param lookup_list A valid HebiLookupEntryList object.
 * @param index The entry index that is being queried.
 * @param buffer An allocated buffer of length 'length'
 * @param length the length of the provided buffer. After calling this function, the value dereferenced will be
 * updated with the length of the string plus the null character.
 *
 * \returns HebiStatusSuccess on success, HebiStatusBufferTooSmall if the provided buffer is too small, or
 * HebiStatusInvalidArgument if the length parameter is null
 */
HebiStatusCode hebiLookupEntryListGetName(HebiLookupEntryListPtr lookup_list, int index, char* buffer, size_t* length);
/**
 * Gets the family of the given entry in the lookup entry list. Must be a valid
 * index.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * Note - assumes ASCII string encoding.
 *
 * @param lookup_list A valid HebiLookupEntryList object.
 * @param index The entry index that is being queried.
 * @param buffer An allocated buffer of length 'length'.
 * @param length the length of the provided buffer. After calling this function, the value dereferenced will be
 * updated with the length of the string plus the null character.
 *
 * \returns HebiStatusSuccess on success, HebiStatusBufferTooSmall if the provided buffer is too small, or
 * HebiStatusInvalidArgument if the length parameter is null
 */
HebiStatusCode hebiLookupEntryListGetFamily(HebiLookupEntryListPtr lookup_list, int index, char* buffer, size_t* length);
/**
 *
 * @param lookup_list A valid HebiLookupEntryList object.
 * @param index The entry index that is being queried.
 */
HebiMacAddress hebiLookupEntryListGetMacAddress(HebiLookupEntryListPtr lookup_list, int index);
/**
 * @brief Release resources for a given lookup entry list; list should not be
 * used after this call.
 *
 * @param lookup_list A valid HebiLookupEntryList object.
 */
void hebiLookupEntryListRelease(HebiLookupEntryListPtr lookup_list);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
