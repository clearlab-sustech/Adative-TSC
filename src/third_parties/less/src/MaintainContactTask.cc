#include "less/MaintainContactTask.h"

namespace clear {

MaintainContactTask::MaintainContactTask(
    shared_ptr<PinocchioInterface> pinocchioInterface_ptr)
    : pinocchioInterface_ptr(pinocchioInterface_ptr) {}

MaintainContactTask::~MaintainContactTask() {}

} // namespace clear
