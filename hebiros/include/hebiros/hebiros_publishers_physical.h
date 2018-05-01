#ifndef HEBIROS_PUBLISHERS_PHYSICAL_H
#define HEBIROS_PUBLISHERS_PHYSICAL_H

#include "hebiros_publishers.h"


class HebirosPublishersPhysical : public HebirosPublishers {

  public:

    void registerGroupPublishers(std::string group_name);

};

#endif
