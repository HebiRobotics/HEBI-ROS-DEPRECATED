#pragma once

#include <cstddef>
#include <iterator>

#include "hebi.h"
#include "mac_address.hpp"
#include "group.hpp"

#include <memory> // For shared_ptr
#include <vector>

namespace hebi {

/**
 * \example lookup_example.cpp
 * How to lookup a group - simple.
 * \example lookup_general_example.cpp
 * How to lookup a group - generalized using command line arguments.
 * \example lookup_helpers.cpp
 * File with parsing of command line arguments and actual group generation used
 * by general examples above.
 */

/**
 * \brief Maintains a registry of network-connected modules and returns Group
 * objects to the user.
 *
 * Only one Lookup object is needed per application.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 18 Feb 2016
 */
class Lookup final
{
  private:
    static const long DEFAULT_TIMEOUT = 500;

    /**
     * \internal C-style lookup object
     */
    HebiLookupPtr lookup_;

  public:
    /**
     * \brief Creates a Lookup object which can create Module and Group
     * references.
     * Typically, only one Lookup object should exist at a time.
     *
     * Note that this call invokes a background thread to query the network for
     * modules at regular intervals.
     */
    Lookup();

    /**
     * \brief Destructor frees all resources created by Lookup object, and stops the
     * background query thread.
     */
    virtual ~Lookup() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * \brief Get a group from modules with the given names and families.
     *
     * If one of the input vectors is of length one, then that element is
     * assumed to pair with each item in the other input vector.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param families A list of families of desired group modules, as viewable
     * in the HEBI GUI.  If of length one, this family is paried with each given
     * name.
     * @param names A list of names of desired group modules, as viewable in the
     * HEBI GUI.  If of length one, this name is paired with each given family
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A shared_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::shared_ptr<Group> getGroupFromNames(const std::vector<std::string>& families, const std::vector<std::string>& names, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from modules with the given mac addresses.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param addresses List of physical mac addresses for desired group modules.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A shared_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::shared_ptr<Group> getGroupFromMacs(const std::vector<MacAddress>& addresses, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from all known modules with the given family.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param family The family of each of the desired group modules.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A shared_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::shared_ptr<Group> getGroupFromFamily(const std::string& family, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from all modules known to connect to a module with the
     * given name and family.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param family The given family of the module, as viewable in the HEBI
     * GUI, to form the group from.
     * @param name The given name of the module, as viewable in the HEBI GUI, to
     * form the group from.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A shared_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::shared_ptr<Group> getConnectedGroupFromName(const std::string& family, const std::string& name, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from all modules known to connect to a module with the
     * given mac address.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param address Physical mac address of the module to form the group from.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A shared_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::shared_ptr<Group> getConnectedGroupFromMac(const MacAddress& address, long timeout_ms=DEFAULT_TIMEOUT);
  
    class EntryList final
    {
      typedef struct Entry final
      {
        std::string name_;
        std::string family_;
        MacAddress mac_address_;
      } Entry;

      private:
        /**
         * \internal C-style lookup entry list object
         */
        HebiLookupEntryListPtr lookup_list_;

        /**
         * \internal Entry list iterator implementation
         * (see http://anderberg.me/2016/07/04/c-custom-iterators/)
         */
        class Iterator final
        {
          public:
            // Iterator traits (not from std::iterator to be C++17 compliant)
            using value_type = Entry;
            using difference_type = int;
            using pointer = Entry*;
            using reference = Entry;
            using iterator_category = std::bidirectional_iterator_tag;

            // Default constructable
            Iterator() = default;
            explicit Iterator(const EntryList* list, size_t current);

            // Dereferencable
            reference operator*() const;

            // Pre- and post-incrementable/decrementable
            Iterator& operator++();
            Iterator operator++(int);
            Iterator& operator--();
            Iterator operator--(int);

            // Equality / inequality
            bool operator==(const Iterator& rhs);
            bool operator!=(const Iterator& rhs);

          private:
            const EntryList* list_;
            size_t current_ { 0 };
        };

      public:

        /**
         * \internal Creates entry list from internal C-style object.
         */
        EntryList(HebiLookupEntryListPtr lookup_list) : lookup_list_(lookup_list) {}

        virtual ~EntryList() noexcept;

        Entry getEntry(int index) const;

        int size() const;

        Iterator begin() const;
        Iterator end() const;

      private:
        /**
         * Disable copy and move constructors and assignment operators
         */
        HEBI_DISABLE_COPY_MOVE(EntryList)
    };

    std::shared_ptr<EntryList> getEntryList();
 
  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(Lookup)
};

} // namespace hebi
