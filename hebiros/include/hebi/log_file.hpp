#pragma once

#include "hebi.h"
#include "group_feedback.hpp"

#include <memory>

namespace hebi {

class LogFile {

  friend class Group;

private:

  HebiLogFilePtr internal_;
  const int number_of_modules_;

  LogFile(HebiLogFilePtr internal, int number_of_modules);

public:

  /**
   * @brief Opens an existing log file
   *
   * @param file The absolute or relative path to an existing log file
   * @return a shared pointer to the opened file on success, otherwise
   * a null pointer (i.e. invalid log file or nonexistent file)
   */
  static std::shared_ptr<LogFile> open(std::string file);

  /**
   * @brief Returns the number of modules in the log file
   */
  int size();

  /**
   * @brief retrieves the next group feedback from the log file, if any exists
   *
   * Warning: other data in the provided 'Feedback' object is erased!
   *
   * @param feedback On success, the group feedback read from the group are written
     * into this structure.
   * @return @c true if feedback was returned, otherwise @c false on failure
   */
  bool getNextFeedback(GroupFeedback* feedback);

  LogFile() = delete;
  ~LogFile();

  HEBI_DISABLE_COPY_MOVE(LogFile);

};

}
