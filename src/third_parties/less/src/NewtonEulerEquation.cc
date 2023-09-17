#include "less/NewtonEulerEquation.h"

namespace clear {

NewtonEulerEquation::NewtonEulerEquation(
    shared_ptr<PinocchioInterface> pinocchioInterface_ptr)
    : pinocchioInterface_ptr(pinocchioInterface_ptr) {}

NewtonEulerEquation::~NewtonEulerEquation() {}

} // namespace clear
