//
// Created by nimapng on 7/23/21.
//

#ifndef XIAOTIANHYBRID_LEGCONTROLLER_H
#define XIAOTIANHYBRID_LEGCONTROLLER_H

#include "DataSets.h"
#include "FootSwingTrajectory.h"

using namespace Eigen;

class LegController {
public:
    LegController(const EstimatedState *estimatedState,
                  const RobotModelData *robotModelData,
                  const TasksData *tasksData,
                  const GaitData *gaitData,
                  JointsCmd *jointsCmd,
                  const UserParameterHandler *param);

    void run();

    void toRobot();

    void swingTest(size_t leg);

private:
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const TasksData *tasksData;
    const GaitData *gaitData;
    JointsCmd *jointsCmd;
    const UserParameterHandler *param;
    bool contactDetection[4];

    /* swing test */
    bool first = true;
    FootSwingTrajectory footSwingTrajectory;
};


#endif //XIAOTIANHYBRID_LEGCONTROLLER_H
