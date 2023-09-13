//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_CONTROLLER_H
#define XIAOTIANHYBRID_CONTROLLER_H

#include "DataSets.h"
#include "WholeBodyController.h"
#include "LegController.h"
#include "TSC.h"

class Controller
{
public:
    Controller(const EstimatedState *estimatedState,
               const RobotModelData *robotModelData,
               const TasksData *tasksData,
               const GaitData *gaitData,
               JointsCmd *jointsCmd,
               const UserParameterHandler *param);

    virtual void run();

private:
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const TasksData *tasksData;
    const GaitData *gaitData;
    JointsCmd *jointsCmd;
    const UserParameterHandler *param;

    WholeBodyController wbc;
    TSC_IMPL tsc;
    LegController legController;

    DVec qpos_init;
    DVec qpos_mid_des;
    DVec qpos_des;

    bool enter_down;
    bool enter_stand;
    int _iter;

    void PDStance();
    void PDsitdown();
};

#endif // XIAOTIANHYBRID_CONTROLLER_H
