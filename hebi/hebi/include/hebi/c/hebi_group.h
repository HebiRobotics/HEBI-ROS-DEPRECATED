#pragma once

#include "hebi_group_command.h"
#include "hebi_group_feedback.h"
#include "hebi_group_info.h"
#include "hebi_log.h"
#include "hebi_lookup.h"
#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * \example command_example.c
 * Sending commands to a group of modules.
 * \example command_persist_settings_example.c
 * How to indicate to a group of modules to save their current settings (so that
 * they persist through reboot).
 * \example command_settings_example.c
 * Sending setting commands to a group of modules.
 * \example command_spring_constants_example.c
 * Sending and checking spring constants for a group of modules.
 * \example command_control_strategy_example.c
 * Sending control strategies for a group of modules.
 * \example feedback_async_example.c
 * Getting feedback from a group through a callback from the API at regular intervals.
 * \example feedback_sync_example.c
 * Getting feedback from a group through blocking calls to the API.
 * \example info_sync_example.c
 * Getting info from a group through blocking calls to the API.
 */

/**
 * \brief The C-style's API representation of a group.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * Represents a connection to a group of modules. Sends commands to and receives
 * feedback from the group.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 25 Mar 2015
 */
typedef struct _HebiGroup* HebiGroupPtr;

/**
 * \brief Define a type for a group feedback handling function.
 */
typedef void (*GroupFeedbackHandlerFunction)(HebiGroupFeedbackPtr, void* user_data);

/**
 * \brief Creates an "imitation" group with the specified number of modules.
 *
 * The imitation group is useful for testing the API, as it acts like a Group
 * would in most cases, but is not backed by hardware.  Commands that are
 * sent to the imitation group are returned as feedback, using the standard
 * feedback request methods.
 *
 * Note that standard groups are created through the HebiLookup objects.
 *
 * \param size The number of modules in the group.
 *
 * \returns An imitation group that returns commanded values as feedback.
 */
HebiGroupPtr hebiGroupCreateImitation(unsigned int size);

/**
 * \brief Create a group of modules with the given MAC addresses.
 *
 * If any given modules are not found, no group is created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * @param lookup A valid HebiLookup object.
 * @param addresses An array of pointers to physical mac addresses of the given
 * modules. Length of the array must equal num_addresses.
 * @param num_addresses Length of the addresses array of pointers (number of
 * pointers in the array, not cumulative size of objects they point to).
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromMacs(HebiLookupPtr lookup, const HebiMacAddress* addresses,
                                                int num_addresses, long timeout_ms);
/**
 * \brief Create a group with modules matching the given names and families.
 *
 * If only one family is given, it is used for all modules.  Otherwise, number of
 * names and families must match. If any given modules are not found, no group is
 * created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * @param lookup A valid HebiLookup object.
 * @param names The given names of the modules, as viewable in the HEBI GUI. Must
 * be a list of pointers to null-terminated strings. The number of pointers must
 * match the num_names parameter.
 * @param num_names The number of pointers to null-terminated strings given
 * by the names parameter.
 * @param families The given families of the modules, as viewable in the HEBI
 * GUI. Must be a list of pointers to null-terminated strings. The number of
 * pointers must match the num_families parameter. Note that a single string
 * (with corresponding value of num_families == 1) will be used with each name in
 * the names list.
 * @param num_families The number of pointers to null-terminated strings given
 * by the families parameter. Note that this must either be 1, or be equal to
 * num_names.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromNames(HebiLookupPtr lookup, const char* const* families, int num_families,
                                                 const char* const* names, int num_names, long timeout_ms);
/**
 * \brief Create a group with all modules known to the lookup with the given family.
 *
 * Group contains all modules with the given family, regardless of name.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * @param lookup A valid HebiLookup object.
 * @param family The given family of the modules, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromFamily(HebiLookupPtr lookup, const char* family, long timeout_ms);
/**
 * \brief Create a group with all modules connected to module with the given MAC
 * address.
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * @param lookup A valid HebiLookup object.
 * @param address Physical mac address of the given module (serves as unique id).
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateConnectedFromMac(HebiLookupPtr lookup, const HebiMacAddress* address,
                                                        long timeout_ms);
/**
 * \brief Create a group with all modules connected to module with the given name
 * and family
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * @param lookup A valid HebiLookup object.
 * @param name The given name of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param family The given family of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateConnectedFromName(HebiLookupPtr lookup, const char* family, const char* name,
                                                         long timeout_ms);

/**
 * \brief Returns the number of modules in a group.
 *
 * \param group The group to send this command to.
 *
 * \returns the number of modules in 'group'.
 */
int hebiGroupGetSize(HebiGroupPtr group);

/**
 * \brief Sends a command to the given group, requesting an acknowledgement of
 * transmission to be sent back.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns HebiStatusSuccess if an acknowledgement was successfully received (guaranteeing
 * the group received this command), or a failure code for an error otherwise.
 *
 * Note: A non-HebiStatusSuccess return does not indicate a specific failure,
 * and may result from an error while sending or simply a timeout/dropped
 * response packet after a successful transmission.
 */
HebiStatusCode hebiGroupSendCommandWithAcknowledgement(HebiGroupPtr group,
  HebiGroupCommandPtr command,
  int timeout_ms);

/**
 * \brief Sends a command to the given group without requesting an
 * acknowledgement.
 *
 * Appropriate for high-frequency applications.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 *
 * \returns HebiStatusSuccess if the command was successfully sent, otherwise a failure code.
 */
HebiStatusCode hebiGroupSendCommand(HebiGroupPtr group,
  HebiGroupCommandPtr command);

/**
 * \brief Sets the command lifetime for the group, in milliseconds.
 *
 * The command lifetime is the duration for which a sent command remains active.
 * If the hardware does not receive further commands within the specified time
 * frame, all local controllers get deactivated. This is a safety feature to
 * mitigate the risk of accidents in case programs get interrupted in an unsafe
 * state, e.g., on program exceptions or during a network fault.
 *
 * Additionally, supporting hardware does not accept commands from any other
 * sources during the lifetime of a command. This mitigates the risk of other
 * users accidentally sending conflicting targets from, e.g., the GUI.
 *
 * \param group Which group the command lifetime is being set for.
 * \param lifetime_ms The number of milliseconds which the command 'lives' for.
 * Setting a value less than or equal to '0' disables command lifetime. When
 * disabled, the hardware will continue to execute the last sent command.
 * Setting a value above the accepted maximum will set the lockout to the
 * maximum value.
 *
 * \returns HebiStatusSuccess if command lifetime successfully set, or a failure code if
 * value was outside of accepted range (higher than supported maximum or negative).
 */
HebiStatusCode hebiGroupSetCommandLifetime(HebiGroupPtr group, int32_t lifetime_ms);

/**
 * \brief Returns the current command lifetime, in milliseconds.
 *
 * \param group Which group is being queried.
 *
 * \returns The current command lifetime, in milliseconds. A value of '0' indicates
 * that commands remain active until the next command is received.
 */
int32_t hebiGroupGetCommandLifetime(HebiGroupPtr group);

/**
 * \brief Sets the feedback request loop frequency (in Hz).
 *
 * The group is queried for feedback in a background thread at this frequency,
 * and any added callbacks are called from this background thread.
 *
 * \param group Which group this frequency set is for.
 * \param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * \returns HebiStatusSuccess if feedback frequency successfully set, or a failure code if
 * value was outside of accepted range (higher than supported maximum, NaN or negative).
 */
HebiStatusCode hebiGroupSetFeedbackFrequencyHz(HebiGroupPtr group, float frequency);

/**
 * \brief Returns the current feedback request loop frequency (in Hz).
 *
 * \param group Which group is being queried.
 *
 * \returns The current feedback request loop frequency (in Hz).
 */
float hebiGroupGetFeedbackFrequencyHz(HebiGroupPtr group);

/**
 * \brief Add a function that is called whenever feedback is returned from the
 * group.
 *
 * \param group The group to attach this handler to.
 * \param handler A feedback handling function called whenever feedback is
 * received from the group.
 * \param user_data A pointer to user data which will be returned as the second
 * callback argument. This pointer can be NULL if desired.
 */
void hebiGroupRegisterFeedbackHandler(
  HebiGroupPtr group, GroupFeedbackHandlerFunction handler, void* user_data);

/**
 * \brief Removes all feedback handling functions from the queue to be called on
 * receipt of group feedback.
 *
 * \param group The group to which the handlers are attached.
 */
void hebiGroupClearFeedbackHandlers(
  HebiGroupPtr group);

/**
 * \brief Requests feedback from the group.
 *
 * Sends a background request to the modules in the group; if/when all modules
 * return feedback, any associated handler functions are called. This returned
 * feedback is also stored to be returned by the next call to
 * hebiGroupGetNextFeedback (any previously returned data is discarded).
 *
 * \param group The group to return feedback from.
 *
 * \returns HebiStatusSuccess if request was successfully sent, or a failure code if not
 * (i.e., connection error).
 */
HebiStatusCode hebiGroupSendFeedbackRequest(HebiGroupPtr group);

/**
 * \brief Returns the most recently stored feedback from a sent feedback
 * request, or returns the next one received (up to the requested timeout).
 *
 * Note that a feedback request can be sent either with the
 * hebiGroupSendFeedbackRequest function, or by setting a background feedback
 * frequency with hebiGroupSetFeedbackFrequencyHz.
 *
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * \param group The group to return feedback from.
 * \param feedback On success, the feedback read from the group are written
 * into this structure.
 * \param timeout_ms Indicates how many milliseconds to wait for feedback.
 * For typical networks, '15' ms is a value that can be reasonably expected to
 * allow for a round trip transmission after the last 'send feedback request'
 * call.
 *
 * \returns HebiStatusSuccess if feedback was returned, or a failure code if not
 * (i.e., connection error or timeout waiting for response).
 */
HebiStatusCode hebiGroupGetNextFeedback(HebiGroupPtr group, HebiGroupFeedbackPtr feedback,
  int timeout_ms);

/**
 * \brief Requests info from the group, and writes it to the provided info
 * object.
 *
 * Warning: other data in the provided 'Info' object is erased!
 *
 * \param group The group to send this command to.
 * \param info On success, the info read from the group is written into this
 * structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns HebiStatusSuccess if info was received, or a failure code if not
 * (i.e., connection error or timeout waiting for response).
 */
HebiStatusCode hebiGroupRequestInfo(HebiGroupPtr group, HebiGroupInfoPtr info,
  int timeout_ms);

/**
 * \brief Starts logging data to a file in the local directory.
 *
 * WARNING: this function interface is not yet stable.
 *
 * \param group The group to log from.
 * \param dir The relative or absolute path to the directory to log in. To use
 * the current directory, pass in a null pointer
 * \param file The optional file name. If this is null, a name will be created using
 * the time at the moment which this function was invoked.
 *
 * \returns HebiStatusSuccess if successfully started a log, a failure code otherwise.
 */
HebiStatusCode hebiGroupStartLog(HebiGroupPtr group, const char* dir, const char* file);

/**
 * \brief Stops logging data to a file in the local directory.
 * Note: This function allocates a log file structure on the heap, so make sure to release the pointer
 * returned by this function by calling @c hebiLogFileRelease(HebiLogFilePtr ptr)
 *
 * \param group The group that is logging.
 *
 * \returns a log file instance on success, otherwise a null pointer.
 */
HebiLogFilePtr hebiGroupStopLog(HebiGroupPtr group);

/**
 * \brief Release resources for a given group; group should not be used after
 * this call.
 *
 * @param group A valid HebiGroup object.
 */
void hebiGroupRelease(HebiGroupPtr group);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
