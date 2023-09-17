#include "less/SE3Task.h"

namespace clear {

SE3Task::SE3Task(shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
                 string name)
    : pinocchioInterface_ptr(pinocchioInterface_ptr), name_(name) {}

SE3Task::~SE3Task() {}

} // namespace clear
