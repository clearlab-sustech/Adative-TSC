//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_ESTIMATESTATE_H
#define XIAOTIANHYBRID_ESTIMATESTATE_H

#include "MeasuredState.h"

struct FloatingBaseState {
    Vec3 pos, rpy, vWorld, vBody, omegaWorld, omegaBody, aBody, aWorld;
    Mat3 R_wb;
    Quat quat;
};

struct FootState {
    Vec3 pos, rpy, vWorld, omegaWorld;
    Mat3 R_wf, R_wh;
    ContactState contactState;
    Vec3 contactPoint;
    int16_t force = 0;
    bool contact_detected = false;
};

enum class Feet {
    FL, FR, HL, HR
};

struct EstimatedState {
    FloatingBaseState floatingBaseState;
    JointsState jointsState;  //change to Jointstate
    FootState footState[4];

    Vec4 contactPhaseDes = Vec4::Zero();
};
#endif //XIAOTIANHYBRID_ESTIMATESTATE_H
