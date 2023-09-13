//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_GAITSCHEDULER_H
#define XIAOTIANHYBRID_GAITSCHEDULER_H

#include "DataSets.h"
#include "FSM.h"
#include "GaitTypes.h"
#include "Gait.h"

#include <fstream>

using namespace std;

#define csvout(outport,x,flag) if(flag){outport << #x <<",";}else{outport<<x<<",";}
#define csvoutN(outport,x,n,flag) if(flag){for(int i=0;i<n;++i){outport << #x << "[" << i <<"]/" << n << ",";}}else{for(int j=0;j<n;++j){outport<<x[j]<<",";}}

class GaitScheduler {
private:
  const EstimatedState *estimatedState;
  const UserCmd *userCmd;
  GaitData *gaitData;
  const UserParameterHandler *param;
  FSM fsm;
  Gait trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
  size_t iter;

  GaitTypes currentGait;

  Gait* gait = nullptr;
  
  bool checkTransition();

public:
  GaitScheduler(const EstimatedState *estimatedState, const UserCmd *userCmd, GaitData *gaitData,
                const UserParameterHandler *param);

  void step();

  ofstream csvLog;
};


#endif //XIAOTIANHYBRID_GAITSCHEDULER_H
