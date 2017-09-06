#include "lookup.hpp"
#include <algorithm> // For std::transform
#include <iterator> // For std::back_inserter on Windows.

namespace hebi {

Lookup::Lookup()
{
  lookup_ = hebiLookupCreate();
}

Lookup::~Lookup() noexcept
{
  hebiLookupRelease(lookup_);
}

std::shared_ptr<Group> Lookup::getGroupFromNames(const std::vector<std::string>& families, const std::vector<std::string>& names, long timeout_ms)
{
  std::shared_ptr<Group> ptr;
  std::vector<const char *> names_cstrs;
  std::vector<const char *> families_cstrs;
  names_cstrs.reserve(names.size());
  families_cstrs.reserve(families.size());

  std::transform(std::begin(names), std::end(names),
    std::back_inserter(names_cstrs), [] (const std::string& name) { return name.c_str(); });
  std::transform(std::begin(families), std::end(families),
    std::back_inserter(families_cstrs), [] (const std::string& family) { return family.c_str(); });

  HebiGroupPtr group = hebiGroupCreateFromNames(lookup_, families_cstrs.data(), families_cstrs.size(), names_cstrs.data(), names_cstrs.size(), timeout_ms);
  if (group != nullptr)
    return std::make_shared<Group>(group);
  return ptr;
}

std::shared_ptr<Group> Lookup::getGroupFromMacs(const std::vector<MacAddress>& addresses, long timeout_ms)
{
  std::shared_ptr<Group> ptr;
  std::vector<HebiMacAddress> addresses_c;
  addresses_c.reserve(addresses.size());
  std::transform(std::begin(addresses), std::end(addresses),
    std::back_inserter(addresses_c), [] (const MacAddress& addr) { return addr.internal_; });
  HebiGroupPtr group = hebiGroupCreateFromMacs(lookup_, addresses_c.data(), addresses.size(), timeout_ms);
  if (group != nullptr)
    return std::make_shared<Group>(group);
  return ptr;
}

std::shared_ptr<Group> Lookup::getGroupFromFamily(const std::string& family, long timeout_ms)
{
  std::shared_ptr<Group> ptr;
  HebiGroupPtr group = hebiGroupCreateFromFamily(lookup_, family.c_str(), timeout_ms);
  if (group != nullptr)
    return std::make_shared<Group>(group);
  return ptr;
}

std::shared_ptr<Group> Lookup::getConnectedGroupFromName(const std::string& family_name, const std::string& name, long timeout_ms)
{
  std::shared_ptr<Group> ptr;
  HebiGroupPtr group = hebiGroupCreateConnectedFromName(lookup_, family_name.c_str(), name.c_str(), timeout_ms);
  if (group != nullptr)
    return std::make_shared<Group>(group);
  return ptr;
}

std::shared_ptr<Group> Lookup::getConnectedGroupFromMac(const MacAddress& address, long timeout_ms)
{
  std::shared_ptr<Group> ptr;
  HebiGroupPtr group = hebiGroupCreateConnectedFromMac(lookup_, &(address.internal_), timeout_ms);
  if (group != nullptr)
    return std::make_shared<Group>(group);
  return ptr;
}

Lookup::EntryList::~EntryList() noexcept
{
  hebiLookupEntryListRelease(lookup_list_);
  lookup_list_ = nullptr;
}

Lookup::EntryList::Entry Lookup::EntryList::getEntry(int index)
{
  size_t required_size = 0;
  hebiLookupEntryListGetName(lookup_list_, index, nullptr, &required_size);
  auto buffer = new char [required_size];
  hebiLookupEntryListGetName(lookup_list_, index, buffer, &required_size);
  std::string name(buffer, required_size-1);
  delete[] buffer;

  required_size = 0;
  hebiLookupEntryListGetFamily(lookup_list_, index, nullptr, &required_size);
  buffer = new char [required_size];
  hebiLookupEntryListGetFamily(lookup_list_, index, buffer, &required_size);
  std::string family(buffer, required_size-1);
  delete[] buffer;

  HebiMacAddress mac_int = hebiLookupEntryListGetMacAddress(lookup_list_, index);
  MacAddress mac;
  mac.internal_ = mac_int;

  Entry e = { name, family, mac };
  return e;
}

int Lookup::EntryList::size()
{
  return hebiLookupEntryListGetSize(lookup_list_);
}

std::shared_ptr<Lookup::EntryList> Lookup::getEntryList()
{
  std::shared_ptr<Lookup::EntryList> ptr;
  auto entry_list = hebiCreateLookupEntryList(lookup_);
  if (entry_list != nullptr)
    return std::make_shared<Lookup::EntryList>(entry_list);
  return ptr;
}

} // namespace hebi
