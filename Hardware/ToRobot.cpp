//
// Created by zhenfu on 2021/12/14.
//

#include "ToRobot.h"

using namespace UNITREE_LEGGED_SDK;

void RealRobot::updateStateData()
{
    this->measuredState->mtx.lock();
    mutex_state.lock();
    this->measuredState->imuData.acc << this->state.imu.accelerometer[0], this->state.imu.accelerometer[1], this->state.imu.accelerometer[2];
    this->measuredState->imuData.gyro << this->state.imu.gyroscope[0], this->state.imu.gyroscope[1], this->state.imu.gyroscope[2];
    this->measuredState->imuData.quat << this->state.imu.quaternion[0], this->state.imu.quaternion[1], this->state.imu.quaternion[2], this->state.imu.quaternion[3];

    this->measuredState->footForce[0] = this->state.footForce[1];
    this->measuredState->footForce[1] = this->state.footForce[0];
    this->measuredState->footForce[2] = this->state.footForce[3];
    this->measuredState->footForce[3] = this->state.footForce[2];

    this->measuredState->jointsState.tau << this->state.motorState[3].tauEst,
        this->state.motorState[4].tauEst,
        this->state.motorState[5].tauEst,
        this->state.motorState[0].tauEst,
        this->state.motorState[1].tauEst,
        this->state.motorState[2].tauEst,
        this->state.motorState[9].tauEst,
        this->state.motorState[10].tauEst,
        this->state.motorState[11].tauEst,
        this->state.motorState[6].tauEst,
        this->state.motorState[7].tauEst,
        this->state.motorState[8].tauEst;
    this->measuredState->jointsState.qpos << this->state.motorState[3].q,
        this->state.motorState[4].q,
        this->state.motorState[5].q,
        this->state.motorState[0].q,
        this->state.motorState[1].q,
        this->state.motorState[2].q,
        this->state.motorState[9].q,
        this->state.motorState[10].q,
        this->state.motorState[11].q,
        this->state.motorState[6].q,
        this->state.motorState[7].q,
        this->state.motorState[8].q;
    this->measuredState->jointsState.qvel << this->state.motorState[3].dq,
        this->state.motorState[4].dq,
        this->state.motorState[5].dq,
        this->state.motorState[0].dq,
        this->state.motorState[1].dq,
        this->state.motorState[2].dq,
        this->state.motorState[9].dq,
        this->state.motorState[10].dq,
        this->state.motorState[11].dq,
        this->state.motorState[6].dq,
        this->state.motorState[7].dq,
        this->state.motorState[8].dq;
    mutex_state.unlock();
    measuredState->isUpdated = true;
    this->measuredState->mtx.unlock();
}

void RealRobot::updateCmdData()
{
    jointCmd->mtx.lock();
    mutex_cmd.lock();
    this->cmd.motorCmd[0].tau = this->jointCmd->tau_ff[3];
    this->cmd.motorCmd[0].Kp = this->jointCmd->Kp[3];
    this->cmd.motorCmd[0].Kd = this->jointCmd->Kd[3];
    this->cmd.motorCmd[0].q = this->jointCmd->qpos_des[3];
    this->cmd.motorCmd[0].dq = this->jointCmd->qvel_des[3];

    this->cmd.motorCmd[1].tau = this->jointCmd->tau_ff[4];
    this->cmd.motorCmd[1].Kp = this->jointCmd->Kp[4];
    this->cmd.motorCmd[1].Kd = this->jointCmd->Kd[4];
    this->cmd.motorCmd[1].q = this->jointCmd->qpos_des[4];
    this->cmd.motorCmd[1].dq = this->jointCmd->qvel_des[4];

    this->cmd.motorCmd[2].tau = this->jointCmd->tau_ff[5];
    this->cmd.motorCmd[2].Kp = this->jointCmd->Kp[5];
    this->cmd.motorCmd[2].Kd = this->jointCmd->Kd[5];
    this->cmd.motorCmd[2].q = this->jointCmd->qpos_des[5];
    this->cmd.motorCmd[2].dq = this->jointCmd->qvel_des[5];

    this->cmd.motorCmd[3].tau = this->jointCmd->tau_ff[0];
    this->cmd.motorCmd[3].Kp = this->jointCmd->Kp[0];
    this->cmd.motorCmd[3].Kd = this->jointCmd->Kd[0];
    this->cmd.motorCmd[3].q = this->jointCmd->qpos_des[0];
    this->cmd.motorCmd[3].dq = this->jointCmd->qvel_des[0];

    this->cmd.motorCmd[4].tau = this->jointCmd->tau_ff[1];
    this->cmd.motorCmd[4].Kp = this->jointCmd->Kp[1];
    this->cmd.motorCmd[4].Kd = this->jointCmd->Kd[1];
    this->cmd.motorCmd[4].q = this->jointCmd->qpos_des[1];
    this->cmd.motorCmd[4].dq = this->jointCmd->qvel_des[1];

    this->cmd.motorCmd[5].tau = this->jointCmd->tau_ff[2];
    this->cmd.motorCmd[5].Kp = this->jointCmd->Kp[2];
    this->cmd.motorCmd[5].Kd = this->jointCmd->Kd[2];
    this->cmd.motorCmd[5].q = this->jointCmd->qpos_des[2];
    this->cmd.motorCmd[5].dq = this->jointCmd->qvel_des[2];

    this->cmd.motorCmd[6].tau = this->jointCmd->tau_ff[9];
    this->cmd.motorCmd[6].Kp = this->jointCmd->Kp[9];
    this->cmd.motorCmd[6].Kd = this->jointCmd->Kd[9];
    this->cmd.motorCmd[6].q = this->jointCmd->qpos_des[9];
    this->cmd.motorCmd[6].dq = this->jointCmd->qvel_des[9];

    this->cmd.motorCmd[7].tau = this->jointCmd->tau_ff[10];
    this->cmd.motorCmd[7].Kp = this->jointCmd->Kp[10];
    this->cmd.motorCmd[7].Kd = this->jointCmd->Kd[10];
    this->cmd.motorCmd[7].q = this->jointCmd->qpos_des[10];
    this->cmd.motorCmd[7].dq = this->jointCmd->qvel_des[10];

    this->cmd.motorCmd[8].tau = this->jointCmd->tau_ff[11];
    this->cmd.motorCmd[8].Kp = this->jointCmd->Kp[11];
    this->cmd.motorCmd[8].Kd = this->jointCmd->Kd[11];
    this->cmd.motorCmd[8].q = this->jointCmd->qpos_des[11];
    this->cmd.motorCmd[8].dq = this->jointCmd->qvel_des[11];

    this->cmd.motorCmd[9].tau = this->jointCmd->tau_ff[6];
    this->cmd.motorCmd[9].Kp = this->jointCmd->Kp[6];
    this->cmd.motorCmd[9].Kd = this->jointCmd->Kd[6];
    this->cmd.motorCmd[9].q = this->jointCmd->qpos_des[6];
    this->cmd.motorCmd[9].dq = this->jointCmd->qvel_des[6];

    this->cmd.motorCmd[10].tau = this->jointCmd->tau_ff[7];
    this->cmd.motorCmd[10].Kp = this->jointCmd->Kp[7];
    this->cmd.motorCmd[10].Kd = this->jointCmd->Kd[7];
    this->cmd.motorCmd[10].q = this->jointCmd->qpos_des[7];
    this->cmd.motorCmd[10].dq = this->jointCmd->qvel_des[7];

    this->cmd.motorCmd[11].tau = this->jointCmd->tau_ff[8];
    this->cmd.motorCmd[11].Kp = this->jointCmd->Kp[8];
    this->cmd.motorCmd[11].Kd = this->jointCmd->Kd[8];
    this->cmd.motorCmd[11].q = this->jointCmd->qpos_des[8];
    this->cmd.motorCmd[11].dq = this->jointCmd->qvel_des[8];

    mutex_cmd.unlock();
    jointCmd->mtx.unlock();
}

void RealRobot::UDPRecv()
{
    udp.Recv();
}

void RealRobot::UDPSend()
{
    udp.Send();
}

void RealRobot::RobotControl()
{
    motiontime++;

    mutex_state.lock();
    udp.GetRecv(state);
    mutex_state.unlock();

    // std::cout << "IMU = ";
    // for (int i = 0; i < 3; i++)
    // {
    //     std::cout << state.imu.accelerometer[i] << " ";
    // }

    // std::cout << std::endl;
    // printf("%d\n", motiontime);
    // gravity compensation
    // cmd.motorCmd[FR_0].tau = -0.65f;
    // cmd.motorCmd[FL_0].tau = +0.65f;
    // cmd.motorCmd[RR_0].tau = -0.65f;
    // cmd.motorCmd[RL_0].tau = +0.65f;

    // if( motiontime >= 500){
    //     float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;
    //     if(torque > 5.0f) torque = 5.0f;
    //     if(torque < -5.0f) torque = -5.0f;

    //     cmd.motorCmd[FR_1].q = PosStopF;
    //     cmd.motorCmd[FR_1].dq = VelStopF;
    //     cmd.motorCmd[FR_1].Kp = 0;
    //     cmd.motorCmd[FR_1].Kd = 0;
    //     cmd.motorCmd[FR_1].tau = torque;
    // }
    mutex_cmd.lock();

    safe.PowerProtect(cmd, state, 10);
    udp.SetSend(cmd);

    mutex_cmd.unlock();
}
