//
// Created by nimapng on 1/29/22.
//

#ifndef CLIMBSTAIRS_POPLARPLANNER_H
#define CLIMBSTAIRS_POPLARPLANNER_H

#include "DataSets.h"

class PoplarPlanner {
public:
    PoplarPlanner(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
            const UserCmd *userCmd, const GaitData *gaitData, TasksData *tasksData,
            const PerceptionData *perceptionData, const UserParameterHandler *param);

    void plan();
    double time_mpc_solved;

private:
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const UserCmd *userCmd;
    const GaitData *gaitData;
    TasksData *tasksData;
    const PerceptionData *perceptionData;
    const UserParameterHandler *param;

    size_t iter;

};


#endif //CLIMBSTAIRS_POPLARPLANNER_H
