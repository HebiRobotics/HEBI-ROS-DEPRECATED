#include "hebiros_group_gazebo.h"


std::map<std::string, std::shared_ptr<HebirosGroupGazebo>> HebirosGroupGazebo::groups_gazebo;

HebirosGroupGazebo::HebirosGroupGazebo(std::string name) :
  HebirosGroup(name) {

  groups_gazebo[name] = std::make_shared<HebirosGroupGazebo>(*this);
}

std::shared_ptr<HebirosGroupGazebo> HebirosGroupGazebo::getGroup(std::string name) {
  return groups_gazebo[name];
}

void HebirosGroupGazebo::removeGroup(std::string name) {
  groups_gazebo.erase(name);
  HebirosGroup::groups.erase(name);
}

bool HebirosGroupGazebo::findGroup(std::string name) {
  return groups_gazebo.find(name) != groups_gazebo.end();
}
