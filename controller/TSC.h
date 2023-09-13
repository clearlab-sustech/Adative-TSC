//
// Created by nimapng on 12/13/21.
//

#ifndef CLIMBSTAIRS_TSC_H
#define CLIMBSTAIRS_TSC_H

#include "DataSets.h"
#include "PoplarLib.h"

using namespace TSC;

class TSC_IMPL {
public:
    TSC_IMPL(const EstimatedState *estimatedState,
             const RobotModelData *robotModelData,
             const TasksData *tasksData,
             const GaitData *gaitData,
             JointsCmd *jointsCmd,
             const UserParameterHandler *param);

    void run();

private:
    void ik();
    
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const TasksData *tasksData;
    const GaitData *gaitData;
    JointsCmd *jointsCmd;
    const UserParameterHandler *param;

    RobotWrapper _robot;
    Vec _lb, _ub;
    SE3MotionTask mt_waist;
    SE3MotionTask fl;
    SE3MotionTask fr;
    SE3MotionTask rl;
    SE3MotionTask rr;

    // CoMMotionTask com;
    RegularizationTask rt;
    ForceTask forceTask;
    // JointsNominalTask jointsNominalTask;
    AngularMomentumTask angularMomentumTask;
    ContactPointsConstraints cpcstr;
    ContactForceConstraints cfcstr;
    ActuatorLimit actuatorLimit;
    TaskSpaceControl tsc;

    vector<string> contact_virtual_link;

    size_t _iter;
};


#endif //CLIMBSTAIRS_TSC_H
