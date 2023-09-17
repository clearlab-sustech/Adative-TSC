#include "less/FrictionCone.h"

namespace clear {

FrictionCone::FrictionCone(
    shared_ptr<PinocchioInterface> pinocchioInterface_ptr)
    : pinocchioInterface_ptr(pinocchioInterface_ptr) {}

FrictionCone::~FrictionCone() {}

} // namespace clear
