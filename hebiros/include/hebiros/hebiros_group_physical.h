#pragma once

#include "group.hpp"
#include "group_info.hpp"
#include "hebiros_group.h"

class HebirosGroupPhysical : public HebirosGroup {

  public:

    std::shared_ptr<hebi::Group> group_ptr;
    hebi::GroupInfo group_info;

    HebirosGroupPhysical(std::shared_ptr<hebi::Group> group);
    virtual ~HebirosGroupPhysical();

    void setFeedbackFrequencyHz(float frequency) override;
    void setCommandLifetimeMs(float lifetime) override;

};
