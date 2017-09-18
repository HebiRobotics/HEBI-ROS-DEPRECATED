#include "log_file.hpp"

namespace hebi {

LogFile::LogFile(HebiLogFilePtr internal, int number_of_modules)
    : internal_(internal),
      number_of_modules_(number_of_modules) {}

std::shared_ptr<LogFile> LogFile::open(std::string file) {
  auto internal = hebiLogFileOpen(file.c_str());
  if (internal == nullptr) {
    return std::shared_ptr<LogFile>();
  }

  return std::shared_ptr<LogFile>(
    new LogFile(internal, hebiLogFileGetNumberOfModules(internal))
  );
}

int LogFile::size() {
  return number_of_modules_;
}

bool LogFile::getNextFeedback(GroupFeedback* feedback) {
  return hebiLogFileGetNextFeedback(internal_, feedback->internal_) == HebiStatusSuccess;
}

LogFile::~LogFile() {
  if (internal_ != nullptr) {
    hebiLogFileRelease(internal_);
  }
}

}