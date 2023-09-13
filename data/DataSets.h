//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_DATASETS_H
#define XIAOTIANHYBRID_DATASETS_H

#include "JointsCmd.h"
#include "EstimatedState.h"
#include "RobotModelData.h"
#include "TaskData.h"
#include "UserCmd.h"
#include "GaitData.h"
#include "Param.h"
#include "PerceptionData.h"

class DataSets {
public:
    DataSets() {
        gaitData.zero();
        perceptionData.planes[0].bound_num = 4;
        perceptionData.planes[0].A << 0, 0, 0;
        perceptionData.planes[0].Bs[0] = Eigen::Vector3d(0, 1, 4.0);
        perceptionData.planes[0].Bs[1] = Eigen::Vector3d(1, 0, 5);
        perceptionData.planes[0].Bs[2] = Eigen::Vector3d(0, -1, 4.0);
        perceptionData.planes[0].Bs[3] = Eigen::Vector3d(-1, 0, 5);
        for (int i = 1; i < 30; i++) {
            perceptionData.planes[i].bound_num = 0;
        }
    }

    MeasuredState measuredState;
    EstimatedState estimatedState;
    RobotModelData robotModelData;
    TasksData tasksData;
    JointsCmd jointsCmd;
    UserCmd userCmd;
    GaitData gaitData;
    PerceptionData perceptionData;
    Location location_vision;
};

#endif //XIAOTIANHYBRID_DATASETS_H
