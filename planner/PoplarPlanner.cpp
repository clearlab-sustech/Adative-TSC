//
// Created by nimapng on 1/29/22.
//

#include "PoplarPlanner.h"

PoplarPlanner::PoplarPlanner(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
                             const UserCmd *userCmd, const GaitData *gaitData, TasksData *tasksData,
                             const PerceptionData *perceptionData, const UserParameterHandler *param) :
        estimatedState(estimatedState),
        robotModelData(robotModelData),
        userCmd(userCmd),
        gaitData(gaitData),
        tasksData(tasksData),
        perceptionData(perceptionData),
        param(param),
        iter(0) {

}

void PoplarPlanner::plan() {

}
