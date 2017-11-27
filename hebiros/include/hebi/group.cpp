#include "group.hpp"
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

Group::Group(HebiGroupPtr group)
  : internal_(group), number_of_modules_(hebiGroupGetSize(internal_))
{
}

std::shared_ptr<Group> Group::createImitation(unsigned int size)
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

bool Group::setCommandLifetimeMs(int ms)
{
  return (hebiGroupSetCommandLifetime(internal_, ms) == HebiStatusSuccess);
}

bool Group::sendCommand(const GroupCommand& group_command)
{
  return (hebiGroupSendCommand(internal_, group_command.internal_) == HebiStatusSuccess);
}

bool Group::sendCommandWithAcknowledgement(const GroupCommand& group_command, int timeout_ms)
{
  return (hebiGroupSendCommandWithAcknowledgement(internal_, group_command.internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::sendFeedbackRequest()
{
  return (hebiGroupSendFeedbackRequest(internal_) == HebiStatusSuccess);
}

bool Group::getNextFeedback(GroupFeedback* feedback, int timeout_ms)
{
  return (hebiGroupGetNextFeedback(internal_, feedback->internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::requestInfo(GroupInfo* info, int timeout_ms)
{
  return (hebiGroupRequestInfo(internal_, info->internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::startLog(std::string dir)
{
  return (hebiGroupStartLog(internal_, dir.c_str(), nullptr) == HebiStatusSuccess);
}

bool Group::startLog(std::string dir, std::string file)
{
  return (hebiGroupStartLog(internal_, dir.c_str(), file.c_str()) == HebiStatusSuccess);
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
