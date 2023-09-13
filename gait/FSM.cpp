//
// Created by wenchun on 3/17/21.
//

#include "FSM.h"

GaitTypes FSM::trans(size_t gaitNum) {
  switch (gaitNum)
  {
  case 0:
    return GaitTypes::NONE;
    break;

  case 1:
    return GaitTypes::STAND;
    break;

  case 2:
    return GaitTypes::TROT_WALK;
    break;

  case 3:
    return GaitTypes::TROT_RUN;
    break;

  case 4:
    return GaitTypes::PACE;
    break;

  case 5:
    return GaitTypes::BOUND;
    break;

  case 6:
    return GaitTypes::ROTARY_GALLOP;
    break;

  case 7:
    return GaitTypes::PRONK;
    break;

  default:
    return GaitTypes::STAND;
    break;
  }
}