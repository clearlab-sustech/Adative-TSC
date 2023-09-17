#pragma once
#include <core/types.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>

using namespace std;

namespace clear {
class TorqueLimits {

public:
  TorqueLimits(shared_ptr<PinocchioInterface> pinocchioInterface_ptr);
  ~TorqueLimits();

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
};

} // namespace clear
