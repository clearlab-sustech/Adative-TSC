//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_JOINTSCMD_H
#define XIAOTIANHYBRID_JOINTSCMD_H

#include "cppTypes.h"
#include <mutex>

struct JointsCmd {
    std::mutex mtx;
    Vec12 Kp = Vec12::Zero();
    Vec12 Kd = Vec12::Zero();
    Vec12 qpos_des = Vec12::Zero();
    Vec12 qvel_des = Vec12::Zero();
    Vec12 tau_ff = Vec12::Zero();
};

#endif //XIAOTIANHYBRID_JOINTSCMD_H
