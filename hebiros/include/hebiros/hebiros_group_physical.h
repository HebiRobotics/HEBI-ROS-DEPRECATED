#ifndef HEBIROS_GROUP_PHYSICAL_H
#define HEBIROS_GROUP_PHYSICAL_H

#include "ros/ros.h"

#include "group.hpp"

#include "hebiros_group.h"


using namespace hebi;

class HebirosGroupPhysical : public HebirosGroup {

  public:

    static std::map<std::string, std::shared_ptr<HebirosGroupPhysical>> groups_physical;

    std::shared_ptr<Group> group_ptr;
    GroupInfo* group_info_ptr;

    HebirosGroupPhysical(std::string name);
    static std::shared_ptr<HebirosGroupPhysical> getGroup(std::string name);
    static void removeGroup(std::string name);
    static bool findGroup(std::string name);

};

#endif
