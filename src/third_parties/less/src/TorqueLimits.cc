#include "less/TorqueLimits.h"

namespace clear {

TorqueLimits::TorqueLimits(
    shared_ptr<PinocchioInterface> pinocchioInterface_ptr)
    : pinocchioInterface_ptr(pinocchioInterface_ptr) {}

TorqueLimits::~TorqueLimits() {}

} // namespace clear
