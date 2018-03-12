
#ifndef _HEBIROS_GAZEBO_GROUP_HH_
#define _HEBIROS_GAZEBO_GROUP_HH_

#include "hebiros_gazebo_joint.h"

using namespace hebiros;

class HebirosGazeboGroup {

  public:

    std::string name;
    std::map<std::string, std::shared_ptr<HebirosGazeboJoint>> joints;

    HebirosGazeboGroup(std::string name);
    ~HebirosGazeboGroup();

};


#endif
