#include "hebiros_group.h"


std::map<std::string, std::shared_ptr<HebirosGroup>> HebirosGroup::groups;

HebirosGroup::HebirosGroup(std::string name) {

  groups[name] = std::make_shared<HebirosGroup>(*this);
}

std::shared_ptr<HebirosGroup> HebirosGroup::getGroup(std::string name) {
  return groups[name];
}

void HebirosGroup::removeGroup(std::string name) {
  groups.erase(name);
}

bool HebirosGroup::findGroup(std::string name) {
  return groups.find(name) != groups.end();
}

