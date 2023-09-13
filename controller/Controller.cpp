//
// Created by wenchun on 3/17/21.
//

#include "Controller.h"

Controller::Controller(const EstimatedState *estimatedState,
                       const RobotModelData *robotModelData,
                       const TasksData *tasksData,
                       const GaitData *gaitData,
                       JointsCmd *jointsCmd,
                       const UserParameterHandler *param) : estimatedState(estimatedState),
                                                            robotModelData(robotModelData),
                                                            tasksData(tasksData),
                                                            gaitData(gaitData),
                                                            jointsCmd(jointsCmd),
                                                            param(param),
                                                            wbc(estimatedState, robotModelData, tasksData, gaitData,
                                                                jointsCmd, param),
                                                            tsc(estimatedState, robotModelData, tasksData, gaitData,
                                                                jointsCmd, param),
                                                            legController(estimatedState, robotModelData, tasksData,
                                                                          gaitData, jointsCmd, param)
{
    qpos_init.resize(12);
    qpos_init.setZero();
    qpos_mid_des.resize(12);
    qpos_mid_des.setZero();
    qpos_des.resize(12);
    qpos_des << 0., 0.67, -1.28,
        0., 0.67, -1.28,
        0., 0.67, -1.28,
        0., 0.67, -1.28;
}

void Controller::run()
{
    static int ctrl_num_pre = 0;
    switch (param->ctrl_num)
    {
    case 0:
        tsc.run();
        break;
    case 1:
        legController.run();
        break;
    case 2: // stance pd
        if (ctrl_num_pre != param->ctrl_num)
            enter_stand = true;
        PDStance();
        break;
    case 3:
        if (ctrl_num_pre != param->ctrl_num)
            enter_down = true;
        PDsitdown();
        break;
    case 8:
        legController.swingTest(1);
        break;
    case 10: // test
        jointsCmd->tau_ff = -5 * estimatedState->jointsState.qpos - 0.1 * estimatedState->jointsState.qvel;
        for (int i = 0; i < 16; i++)
        {
            if (jointsCmd->tau_ff[i] > 5.0)
            {
                jointsCmd->tau_ff[i] = 5.0;
            }
            else if (jointsCmd->tau_ff[i] < -5.0)
            {
                jointsCmd->tau_ff[i] = -5.0;
            }
        }
        break;
    case 11:
        jointsCmd->tau_ff.setZero();
        jointsCmd->Kp.setZero();
        jointsCmd->qvel_des.setZero();
        jointsCmd->Kd.fill(0.1);

        break;
    default:
        throw std::runtime_error("selected controller is not implemented ...");
    }
    ctrl_num_pre = param->ctrl_num;
    // _iter++;
}

void Controller::PDStance()
{
    static int iter;
    if (enter_stand)
    {
        iter = 0;
        enter_stand = false;
        qpos_init = estimatedState->jointsState.qpos;
        qpos_mid_des = estimatedState->jointsState.qpos;
        for (int i(0); i < 4; i++)
            qpos_mid_des[3 * i] = 0.;
    }

    for (int i = 0; i < 4; i++)
    {
        jointsCmd->Kp.segment(3 * i, 3) << 80, 200, 400;
        jointsCmd->Kd.segment(3 * i, 3) << 1.0, 2.0, 5.0;
    }
    DVec qdes;
    double r;
    if (iter <= 500)
    {
        r = double(iter) / 500;
        if (r >= 1)
            r = 1;
        qdes = (1 - r) * qpos_init + r * qpos_mid_des;
    }
    else if (iter > 500 && iter < 3500)
    {
        r = double(iter - 500) / 3000;
        if (r >= 1)
            r = 1;
        qdes = (1 - r) * qpos_mid_des + r * qpos_des;
    }
    else
    {
        // qdes = qpos_mid_des; //TODO
        qdes = qpos_des;
    }
    // if (_iter >= 2500)
    // std::cout << "iter: " << _iter << std::endl;
    // std::cout << "qdes: " << qdes.transpose() << std::endl;
    for (int i(0); i < 12; i++)
    {
        double tau = jointsCmd->Kp[i] * (qdes(i) - estimatedState->jointsState.qpos[i]) +
                     jointsCmd->Kd[i] * (0 - estimatedState->jointsState.qvel[i]);
        if (abs(tau) > 20.0)
        {
            jointsCmd->tau_ff[i] = 20.0 * abs(tau) / tau;
        }
        else
        {
            jointsCmd->tau_ff[i] = tau;
        }
    }
    iter++;
}

void Controller::PDsitdown()
{
    static DVec qpos_cur;
    static int iter_count;
    if (enter_down)
    {
        iter_count = 0;
        enter_down = false;
        qpos_cur = estimatedState->jointsState.qpos;
    }
    for (int i = 0; i < 4; i++)
    {
        jointsCmd->Kp.segment(3 * i, 3) << 80, 200, 400;
        jointsCmd->Kd.segment(3 * i, 3) << 1.0, 2.0, 5.0;
    }
    DVec qdes;
    double r;
    r = double(iter_count) / 2000;
    if (r > 1)
        r = 1.0;

    qdes = (1 - r) * qpos_cur + r * qpos_init;

    // if (_iter >= 2500)
    // std::cout << "iter: " << _iter << std::endl;
    // std::cout << "qdes: " << qdes.transpose() << std::endl;
    for (int i(0); i < 12; i++)
    {
        double tau = jointsCmd->Kp[i] * (qdes(i) - estimatedState->jointsState.qpos[i]) +
                     jointsCmd->Kd[i] * (0 - estimatedState->jointsState.qvel[i]);
        if (abs(tau) > 20)
        {
            jointsCmd->tau_ff[i] = 20 * abs(tau) / tau;
        }
        else
        {
            jointsCmd->tau_ff[i] = tau;
        }
    }
    iter_count++;
}