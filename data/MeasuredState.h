//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_MEASUREDSTATE_H
#define XIAOTIANHYBRID_MEASUREDSTATE_H

#include "cppTypes.h"
#include <mutex>

struct IMUData {
    Quat quat;
    Vec3 gyro;
    Vec3 acc;
};

struct JointsState {
    Vec12 qpos;
    Vec12 qvel;
    Vec12 tau;
};

struct MeasuredState {
    std::mutex mtx;
    bool isUpdated = false;
    IMUData imuData; // [w x y z]
    JointsState jointsState;
    int16_t footForce[4];
};


#endif //XIAOTIANHYBRID_MEASUREDSTATE_H
