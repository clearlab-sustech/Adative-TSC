#pragma once
#include <core/types.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>
#include <string>

using namespace std;
namespace clear {
class SE3Task {
public:
  SE3Task(shared_ptr<PinocchioInterface> pinocchioInterface_ptr, string name);
  ~SE3Task();

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
  string name_;
};


} // namespace clear
