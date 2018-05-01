#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "group_info.hpp"
#include "log_file.hpp"

namespace hebi {

#ifndef DOXYGEN_OMIT_INTERNAL
void callbackWrapper(HebiGroupFeedbackPtr group_feedback, void* user_data)
{
  reinterpret_cast<Group*>(user_data)->callAttachedHandlers(group_feedback);
}
#endif // DOXYGEN_OMIT_INTERNAL

void Group::callAttachedHandlers(HebiGroupFeedbackPtr group_feedback)
{
  // Wrap this:
  GroupFeedback wrapped_fbk(group_feedback);
  // Call handlers:
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  for (unsigned int i = 0; i < handlers_.size(); i++)
  {
    GroupFeedbackHandler handler = handlers_[i];
    try
    {
      handler(wrapped_fbk);
    }
    catch (...)
    {

    }
  }
}

Group::Group(HebiGroupPtr group,
             float initial_feedback_frequency,
             int32_t initial_command_lifetime)
  : internal_(group), number_of_modules_(hebiGroupGetSize(internal_))
{
  if (initial_feedback_frequency != 0)
    setFeedbackFrequencyHz(initial_feedback_frequency);
  if (initial_command_lifetime != 0)
    setCommandLifetimeMs(initial_command_lifetime);
}

std::shared_ptr<Group> Group::createImitation(size_t size)
{
  return std::make_shared<Group>(hebiGroupCreateImitation(size));
}

Group::~Group() noexcept
{
  // Cleanup group object allocated by the C library
  if (internal_ != nullptr)
    hebiGroupRelease(internal_);
}

int Group::size()
{
  return number_of_modules_;
}

bool Group::setCommandLifetimeMs(int32_t ms)
{
  return (hebiGroupSetCommandLifetime(internal_, ms) == HebiStatusSuccess);
}

bool Group::sendCommand(const GroupCommand& group_command)
{
  return (hebiGroupSendCommand(internal_, group_command.internal_) == HebiStatusSuccess);
}

bool Group::sendCommandWithAcknowledgement(const GroupCommand& group_command, int32_t timeout_ms)
{
  return (hebiGroupSendCommandWithAcknowledgement(internal_, group_command.internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::sendFeedbackRequest()
{
  return (hebiGroupSendFeedbackRequest(internal_) == HebiStatusSuccess);
}

bool Group::getNextFeedback(GroupFeedback& feedback, int32_t timeout_ms)
{
  return (hebiGroupGetNextFeedback(internal_, feedback.internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::requestInfo(GroupInfo& info, int32_t timeout_ms)
{
  return (hebiGroupRequestInfo(internal_, info.internal_, timeout_ms) == HebiStatusSuccess);
}

std::string Group::startLog(const std::string& dir)
{
  HebiStringPtr str;
  if (hebiGroupStartLog(internal_, dir.c_str(), nullptr, &str) == HebiStatusSuccess)
  {
    assert(str);
    size_t len;

    hebiStringGetString(str, nullptr, &len);
    char* buffer = new char[len];
    hebiStringGetString(str, buffer, &len);
    std::string ret(buffer, --len);

    delete[] buffer;
    hebiStringRelease(str);

    return ret;
  }
  return "";
}

std::string Group::startLog(const std::string& dir, const std::string& file)
{
  HebiStringPtr str;
  if (hebiGroupStartLog(internal_, dir.c_str(), file.c_str(), &str) == HebiStatusSuccess)
  {
    assert(str);
    size_t len;

    hebiStringGetString(str, nullptr, &len);
    char* buffer = new char[len];
    hebiStringGetString(str, buffer, &len);
    std::string ret(buffer, --len);

    delete[] buffer;
    hebiStringRelease(str);

    return ret;
  }
  return "";
}

std::shared_ptr<LogFile> Group::stopLog()
{
  auto internal = hebiGroupStopLog(internal_);
  if (internal == nullptr) {
    return std::shared_ptr<LogFile>();
  }

  return std::shared_ptr<LogFile>(
    new LogFile(internal, hebiLogFileGetNumberOfModules(internal))
  );
}

bool Group::setFeedbackFrequencyHz(float frequency)
{
  return (hebiGroupSetFeedbackFrequencyHz(internal_, frequency) == HebiStatusSuccess);
}

float Group::getFeedbackFrequencyHz()
{
  return hebiGroupGetFeedbackFrequencyHz(internal_);
}

void Group::addFeedbackHandler(GroupFeedbackHandler handler)
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.push_back(handler);
  if (handlers_.size() == 1) // (i.e., this was the first one)
    hebiGroupRegisterFeedbackHandler(internal_, callbackWrapper, reinterpret_cast<void*>(this));
}

void Group::clearFeedbackHandlers()
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  hebiGroupClearFeedbackHandlers(internal_);
  handlers_.clear();
}

} // namespace hebi
