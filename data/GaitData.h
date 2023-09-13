//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_GAITDATA_H
#define XIAOTIANHYBRID_GAITDATA_H

#include "cppTypes.h"

struct GaitData {
    void zero() {
        swingTime.setZero();
        nextStanceTime.setZero();
        swingTimeRemain.setZero();
        stanceTimeRemain.setZero();
        for(int i=0;i < 4; i++)
        {
            late_contact[i] = false;
        }
    }

    Vec4 swingTime; // keep constant for each leg
    Vec4 nextStanceTime; // may change
    Vec4 swingTimeRemain;
    Vec4 stanceTimeRemain;
    bool late_contact[4];
};

#endif //XIAOTIANHYBRID_GAITDATA_H
