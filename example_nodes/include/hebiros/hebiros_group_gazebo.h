#ifndef HEBIROS_GROUP_GAZEBO_H
#define HEBIROS_GROUP_GAZEBO_H

#include "ros/ros.h"

#include "hebiros_group.h"


class HebirosGroupGazebo : public HebirosGroup {

  public:

    static std::map<std::string, std::shared_ptr<HebirosGroupGazebo>> groups_gazebo;

    HebirosGroupGazebo(std::string name);
    static std::shared_ptr<HebirosGroupGazebo> getGroup(std::string name);
    static void removeGroup(std::string name);
    static bool findGroup(std::string name);


};

#endif
