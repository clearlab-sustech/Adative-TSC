//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_FSM_H
#define XIAOTIANHYBRID_FSM_H

#include "DataSets.h"
#include "GaitTypes.h"

class FSM {
public:
  FSM() = default;
  GaitTypes trans(size_t gaitNum);
};


#endif //XIAOTIANHYBRID_FSM_H
