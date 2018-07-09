#include "hebiros_group_physical.h"
#include "hebiros_group_registry.h"

HebirosGroupPhysical::HebirosGroupPhysical(std::shared_ptr<hebi::Group> group) :
  HebirosGroup(), group_ptr(group), group_info(group->size()) {
}

HebirosGroupPhysical::~HebirosGroupPhysical() {
  group_ptr->clearFeedbackHandlers();
}

void HebirosGroupPhysical::setFeedbackFrequency(float frequency_hz) {
  group_ptr->setFeedbackFrequencyHz(frequency_hz);
}
    
void HebirosGroupPhysical::setCommandLifetime(float lifetime_ms) {
  group_ptr->setCommandLifetimeMs(lifetime_ms);
}
