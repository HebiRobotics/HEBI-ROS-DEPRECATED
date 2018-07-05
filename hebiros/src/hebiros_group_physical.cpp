#include "hebiros_group_physical.h"


std::map<std::string, std::shared_ptr<HebirosGroupPhysical>> HebirosGroupPhysical::groups_physical;

HebirosGroupPhysical::HebirosGroupPhysical(std::string name) :
  HebirosGroup(name) {

  groups_physical[name] = std::make_shared<HebirosGroupPhysical>(*this);
}

std::shared_ptr<HebirosGroupPhysical> HebirosGroupPhysical::getGroup(std::string name) {
  return groups_physical[name];
}

void HebirosGroupPhysical::removeGroup(std::string name) {
  groups_physical.erase(name);
  HebirosGroup::groups.erase(name);
}

bool HebirosGroupPhysical::findGroup(std::string name) {
  return groups_physical.find(name) != groups_physical.end();
}
