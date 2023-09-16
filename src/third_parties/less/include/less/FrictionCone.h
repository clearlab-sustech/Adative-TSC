#pragma once

#include <core/types.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>

using namespace std;

namespace clear {
class FrictionCone {

public:
  FrictionCone(shared_ptr<PinocchioInterface> pinocchioInterface_ptr);
  ~FrictionCone();

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
};

inline FrictionCone::FrictionCone(
    shared_ptr<PinocchioInterface> pinocchioInterface_ptr)
    : pinocchioInterface_ptr(pinocchioInterface_ptr) {}

inline FrictionCone::~FrictionCone() {}

} // namespace clear
