//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_ROBOTMODELDATA_H
#define XIAOTIANHYBRID_ROBOTMODELDATA_H

#include "cppTypes.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/math/rpy.hpp"

#include <iostream>

namespace pin = pinocchio;

struct RobotModelData {
    DMat M;
    DVec bias;
    DVec generalizedGravity;

    DMat Je[4];
    DMat Jc[4];

    DVec Je_dot_q_dot[4];
    DVec Jc_dot_q_dot[4];

    pin::Model model;
    pin::Data data;

    size_t flwID, frwID, hlwID, hrwID;
    size_t flhID, frhID, hlhID, hrhID;
    size_t baseID;
};

#endif //XIAOTIANHYBRID_ROBOTMODELDATA_H
