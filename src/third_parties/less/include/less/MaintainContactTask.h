#pragma once
#include <core/types.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>

using namespace std;
namespace clear {
class MaintainContactTask {

public:
  MaintainContactTask(shared_ptr<PinocchioInterface> pinocchioInterface_ptr);
  ~MaintainContactTask();

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
};

} // namespace clear
