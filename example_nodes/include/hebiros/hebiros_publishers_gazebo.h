#ifndef HEBIROS_PUBLISHERS_GAZEBO_H
#define HEBIROS_PUBLISHERS_GAZEBO_H

#include "hebiros/CommandMsg.h"

#include "hebiros_publishers.h"


class HebirosPublishersGazebo : public HebirosPublishers {

  public:

    void registerGroupPublishers(std::string group_name);
    void command(hebiros::CommandMsg command_msg, std::string group_name);

};

#endif
