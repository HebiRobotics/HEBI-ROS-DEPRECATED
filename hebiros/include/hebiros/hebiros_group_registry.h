#pragma once

#include "hebiros_group.h"

namespace hebiros {

  // This class contains a list of named groups.
  class HebirosGroupRegistry {
  public:
    // TODO: right now this is a singleton during refactoring; this should be
    // changed and owned by the node at a later point, and the singleton removed
    static HebirosGroupRegistry& Instance();

    // Takes ownership of this group, and adds it to the registry of groups.
    void addGroup(const std::string& name, std::unique_ptr<HebirosGroup> group);

    // Returns nullptr on "not found". Ownership is not transferred!
    HebirosGroup* getGroup(const std::string& name); // Should I just make this const instead?
    // Returns nullptr on "not found". Ownership is not transferred!
    const HebirosGroup* getGroup(const std::string& name) const;

    void removeGroup(const std::string& name);

    bool hasGroup(const std::string& name) const;

  private:

    // Note -- after refactoring so this isn't a singleton, this should probably
    // become public.
    HebirosGroupRegistry() = default;

    std::map<std::string, std::unique_ptr<HebirosGroup>> _groups{};

    static HebirosGroupRegistry _instance;

  };

} // namespace hebiros
