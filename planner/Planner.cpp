//
// Created by wenchun on 3/17/21.
//

#include "Planner.h"
#include "Timer.h"

Planner::Planner(const EstimatedState *estimatedState,
                 const RobotModelData *robotModelData,
                 const UserCmd *userCmd,
                 const GaitData *gaitData,
                 TasksData *tasksData,
                 const PerceptionData *perceptionData,
                 const UserParameterHandler *param) :
        estimatedState(estimatedState),
        robotModelData(robotModelData),
        userCmd(userCmd),
        gaitData(gaitData),
        tasksData(tasksData),
        perceptionData(perceptionData),
        param(param),
        footstepPlanner(estimatedState, robotModelData, userCmd, gaitData, 
                        tasksData->footTaskData, &tasksData->comTaskData, 
                        perceptionData, param),
        comPlanner(estimatedState, robotModelData, userCmd, gaitData, tasksData->footTaskData,
                   &tasksData->forcesTaskData,
                   &tasksData->comTaskData, param),
        iter(0) {

}

void Planner::plan() {

    footstepPlanner.plan();

    comPlanner.generateHighLevelRef();

    if (iter % 20 == 0) {
        Timer t;
        comPlanner.plan();
        time_mpc_solved = t.getMs();
    }
    comPlanner.updateTaskData();

    iter++;
}