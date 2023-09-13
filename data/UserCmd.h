//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_USERCMD_H
#define XIAOTIANHYBRID_USERCMD_H

#include "cppTypes.h"

struct UserCmd {
    size_t gaitNum = 1;
    double vx_des = 0;
    double vy_des = 0;
    double yawd_des = 0;
};

#endif //XIAOTIANHYBRID_USERCMD_H
