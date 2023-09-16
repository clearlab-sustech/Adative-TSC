#pragma once
#include <core/types.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>

using namespace std;
namespace clear {
class NewtonEulerEquation {

public:
  NewtonEulerEquation(shared_ptr<PinocchioInterface> pinocchioInterface_ptr);
  ~NewtonEulerEquation();

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
};

NewtonEulerEquation::NewtonEulerEquation(
    shared_ptr<PinocchioInterface> pinocchioInterface_ptr)
    : pinocchioInterface_ptr(pinocchioInterface_ptr) {}

NewtonEulerEquation::~NewtonEulerEquation() {}

} // namespace clear
