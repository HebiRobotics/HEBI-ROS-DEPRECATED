#include "hebiros_group_physical.h"
#include "hebiros_group_registry.h"

HebirosGroupPhysical::HebirosGroupPhysical(std::shared_ptr<hebi::Group> group) :
  HebirosGroup(), group_ptr(group), group_info(group->size()) {
}

HebirosGroupPhysical::~HebirosGroupPhysical() {
  group_ptr->clearFeedbackHandlers();
}

void HebirosGroupPhysical::setFeedbackFrequencyHz(float frequency) {
  group_ptr->setFeedbackFrequencyHz(frequency);
}
    
void HebirosGroupPhysical::setCommandLifetimeMs(float lifetime) {
  group_ptr->setCommandLifetimeMs(lifetime);
}
