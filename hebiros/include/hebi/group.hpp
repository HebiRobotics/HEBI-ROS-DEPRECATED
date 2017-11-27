#pragma once

#include "hebi.h"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "group_info.hpp"
#include "util.hpp"

#include <functional>
#include <memory>
#include <mutex>

namespace hebi {

/**
 * \example command_control_strategy_example.cpp
 *
 * \example command_example.cpp
 *
 * \example command_settings_example.cpp
 *
 * \example feedback_async_example.cpp
 *
 * \example feedback_sync_example.cpp
 *
 * \example info_sync_example.cpp
 *
 * \example master_slave_example.cpp
 *
 * \example master_slave_async_example.cpp
 *
 * \example command_spring_constants_example.cpp
 *
 */

/**
 * \brief Definition of a callback function for GroupFeedback returned from a 
 * Group of modules.
 */
typedef std::function<void (const GroupFeedback&)> GroupFeedbackHandler;

class LogFile;

/**
 * \brief Represents a group of physical HEBI modules, and allows Command,
 * Feedback, and Info objects to be sent to and recieved from the hardware.
 */
class Group final
{
  private:
    /**
     * C-style group object
     */
    HebiGroupPtr internal_;

    /**
     * The number of modules in this group.
     */
    const int number_of_modules_;

    /**
     * Protects access to the group feedback handler vector.
     */
    std::mutex handler_lock_;

    /**
     * A list of handler functions that are called when the internal C API
     * feedback callback is made.
     */
    std::vector<GroupFeedbackHandler> handlers_;

    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * Intermediary to convert C-style function callbacks to C++ style, and
     * change callback parameter types.
     */
    friend void callbackWrapper(HebiGroupFeedbackPtr group_feedback, void* user_data);
    #endif // DOXYGEN_OMIT_INTERNAL

    /**
     * Calls any attached C++ handler functions.  Should only be called from the
     * internal C thread via the callbackWrapper translation function.
     */
    void callAttachedHandlers(HebiGroupFeedbackPtr group_feedback);

  public:
    /**
     * \brief The default timeout for any send-with-acknowledgement or request
     * operation is 500 ms.
     */
    static const int DEFAULT_TIMEOUT_MS = 500;

    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * Creates a group from the underlying C-style group object. This should
     * only be called to create groups from the lookup class, not from user
     * code!
     */
    Group(HebiGroupPtr group);
    #endif // DOXYGEN_OMIT_INTERNAL

    /**
     * \brief Destructor cleans up group.
     */
    virtual ~Group() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * \brief Returns the number of modules in the group
     */
    int size();

    /**
     * \brief Sets the command lifetime for the modules in this group.
     *
     * This parameter defines how long a module will execute a command set
     * point sent to it.  Note the commands from other systems/applications
     * are ignored during this time.  A value of '0' indicates commands last
     * forever, and there is no lockout behavior.
     *
     * See docs.hebi.us for more information.
     */
    bool setCommandLifetimeMs(int ms);

    /**
     * \brief Send a command to the given group without requesting an
     * acknowledgement.
     *
     * Appropriate for high-frequency applications.
     * 
     * \param group_command The GroupCommand object containing information to be
     * sent to the group.
     *
     * \returns true if the command was successfully sent, false otherwise.
     */
    bool sendCommand(const GroupCommand& group_command);

    /**
     * \brief Send a command to the given group, requesting an acknowledgement
     * of transmission to be sent back.
     *
     * \param group_command The GroupCommand object containing information to be
     * sent to the group.
     * \param timeout_ms Indicates how many milliseconds to wait for a response
     * after sending a packet.  For typical networks, '15' ms is a value that
     * can be reasonably expected to encompass the time required for a
     * round-trip transmission.
     *
     * \returns true if an acknowledgement was successfully received
     * (guaranteeing the group received this command), or a negative number for
     * an error otherwise.
     *
     * Note: A non-true return does not indicate a specific failure, and may
     * result from an error while sending or simply a timeout/dropped response
     * packet after a successful transmission.
     */
    bool sendCommandWithAcknowledgement(const GroupCommand& group_command, int timeout_ms=DEFAULT_TIMEOUT_MS);

    /**
     * \brief Requests feedback from the group.
     *
     * Sends a background request to the modules in the group; if/when all modules
     * return feedback, any associated handler functions are called. This returned
     * feedback is also stored to be returned by the next call to
     * getNextFeedback (any previously returned data is discarded).
     *
     * \returns @c true if feedback was request was successfully sent, otherwise @c false on failure
     * (i.e., connection error).
     */
    bool sendFeedbackRequest();

    /**
     * \brief Returns the most recently stored feedback from a sent feedback
     * request, or returns the next one received (up to the requested timeout).
     *
     * Note that a feedback request can be sent either with the
     * sendFeedbackRequest function, or by setting a background feedback
     * frequency with setFeedbackFrequencyHz.
     *
     * Warning: other data in the provided 'Feedback' object is erased!
     *
     * \param feedback On success, the group feedback read from the group are written
     * into this structure.
     * \param timeout_ms Indicates how many milliseconds to wait for feedback.
     * For typical networks, '15' ms is a value that can be reasonably expected to
     * allow for a round trip transmission after the last 'send feedback request'
     * call.
     *
     * \returns @c true if feedback was returned, otherwise @c false on failure (i.e., connection
     * error or timeout waiting for response).
     */
    bool getNextFeedback(GroupFeedback* feedback, int timeout_ms=DEFAULT_TIMEOUT_MS);

    /** 
     * \brief Request info from the group, and store it in the passed-in info
     * object.
     *
     * \returns true if the request was successful within the specified timeout;
     * in this case 'info' has been updated. Otherwise, returns false and does
     * not update 'info'.
     */
    bool requestInfo(GroupInfo* info, int timeout_ms=DEFAULT_TIMEOUT_MS);

    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Starts log (stopping any active log).
     *
     * \param dir The relative or absolute path to the directory to log in. To
     * use the current directory, just use an empty string.
     *
     * \returns true on success.
     */
    bool startLog(std::string dir);

    /**
     * \brief Starts log (stopping any active log).
     *
     * \param dir The relative or absolute path to the directory to log in. To
     * use the current directory, just use an empty string.
     *
     * \param file The name of the file within the directory
     *
     * \returns true on success.
     */
    bool startLog(std::string dir, std::string file);

    /**
     * \brief Stops any active log.
     *
     * \returns shared pointer to the created log file.
     * If the file was not successfully created, this will return null.
     */
    std::shared_ptr<LogFile> stopLog();
    #endif // DOXYGEN_OMIT_INTERNAL

    /**
     * \brief Sets the frequency of the internal feedback request + callback thread.
     *
     * \returns @c true if the frequency successfully was set, or @c false
     * if the parameter was outside the accepted range (less than zero or faster
     * than supported maximum).
     */
    bool setFeedbackFrequencyHz(float frequency);
    /**
     * \brief Gets the frequency of the internal feedback request + callback thread.
     *
     * \returns The current feedback request loop frequency (in Hz).
     */
    float getFeedbackFrequencyHz();
    /**
     * \brief Adds a handler function to be called by the internal feedback request
     * thread.
     */
    void addFeedbackHandler(GroupFeedbackHandler handler);
    /**
     * \brief Removes all feedback handlers presently added.
     */
    void clearFeedbackHandlers();

    /**
     * \brief Creates an imitation group of provided size
     * 
     * \param size the number of modules in the group
     * 
     * \returns A shared pointer to the created imitation group
     */
    static std::shared_ptr<Group> createImitation(unsigned int size);

  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(Group)
};

} // namespace hebi
