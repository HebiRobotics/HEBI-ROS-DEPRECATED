#include "hebiros_group_registry.h"

namespace hebiros {

  // TODO: temporary! remove!
  HebirosGroupRegistry& HebirosGroupRegistry::Instance()
  {
    static HebirosGroupRegistry s;
    return s;
  }

  void HebirosGroupRegistry::addGroup(const std::string& name, std::unique_ptr<HebirosGroup> group) {
    _groups[name] = std::move(group);
  }

  HebirosGroup* HebirosGroupRegistry::getGroup(const std::string& name) {
    if (hasGroup(name))
      return &*_groups[name];
    return nullptr;
  }

  const HebirosGroup* HebirosGroupRegistry::getGroup(const std::string& name) const {
    if (hasGroup(name))
      return &*(_groups.find(name)->second);
    return nullptr;
  }

  void HebirosGroupRegistry::removeGroup(const std::string& name) {
    _groups.erase(name);
  }

  bool HebirosGroupRegistry::hasGroup(const std::string& name) const {
    return _groups.find(name) != _groups.end();
  }

} // namespace hebiros
