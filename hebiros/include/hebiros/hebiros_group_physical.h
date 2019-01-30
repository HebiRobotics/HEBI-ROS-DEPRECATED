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

    void setFeedbackFrequency(float frequency_hz) override;
    void setCommandLifetime(float lifetime_ms) override;

};
