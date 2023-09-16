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

SE3Task::SE3Task(shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
                 string name)
    : pinocchioInterface_ptr(pinocchioInterface_ptr), name_(name) {}

SE3Task::~SE3Task() {}

} // namespace clear
