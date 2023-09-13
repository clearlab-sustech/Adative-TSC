//
// Created by zhenfu on 2021/12/14.
//

#ifndef CLIMBSTAIRS_TOROBOT_H
#define CLIMBSTAIRS_TOROBOT_H

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include "DataSets.h"
#include <mutex>

class RealRobot
{
public:
    RealRobot(uint8_t level, MeasuredState *measuredState, JointsCmd *jointCmd): safe(UNITREE_LEGGED_SDK::LeggedType::Aliengo), measuredState(measuredState),jointCmd(jointCmd),udp(level){
        udp.InitCmdData(cmd);
    }

    void updateStateData();
    void updateCmdData();
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    MeasuredState *measuredState;
    JointsCmd *jointCmd;

    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::UDP udp;
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};
    UNITREE_LEGGED_SDK::LowState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    mutex mutex_state;
    mutex mutex_cmd;
};

#endif //CLIMBSTAIRS_TOROBOT_H
