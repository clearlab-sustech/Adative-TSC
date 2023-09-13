//
// Created by nimapng on 7/23/21.
//

#include "LegController.h"

#define FIX_WHEEL

LegController::LegController(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
                             const TasksData *tasksData, const GaitData *gaitData, JointsCmd *jointsCmd,
                             const UserParameterHandler *param) : estimatedState(estimatedState),
                                                                  robotModelData(robotModelData),
                                                                  tasksData(tasksData),
                                                                  gaitData(gaitData),
                                                                  jointsCmd(jointsCmd),
                                                                  param(param)
{
    for (int i = 0; i < 4; i++)
    {
        contactDetection[i] = false;
    }
}

void LegController::run()
{
    Matrix<double, 12, 18> Jc_all;
    Vec12 force_ref = tasksData->forcesTaskData.forces_ref;
    for (int i = 0; i < 4; i++)
    {
        Jc_all.block(3 * i, 0, 3, 18) = robotModelData->Je[i]; // TODO Jc
        if (0.0 < gaitData->swingTimeRemain[i] && gaitData->swingTimeRemain[i] < 0.03)
        {
            force_ref(3 * i + 2) = 20;
        }
    }

    jointsCmd->tau_ff = -Jc_all.block(0, 6, 12, 12).transpose() * force_ref + robotModelData->bias.tail(12);

    Vec4Int footID;
    footID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;

    DMat Mf = robotModelData->M.topLeftCorner<6, 6>();
    Vec6 qddot_f;
    qddot_f << estimatedState->floatingBaseState.R_wb.transpose() * tasksData->comTaskData.linAccWorld, 0, 0, 0;
    for (int i = 0; i < 4; i++)
    {
        if (gaitData->swingTimeRemain[i] > 0.) // swing
        {
            if (gaitData->swingTimeRemain[i] > 0.02 || gaitData->late_contact[i])
            {
                double swingPhase = 1 - gaitData->swingTimeRemain[i] / gaitData->swingTime[i];
                if (estimatedState->footState[i].contact_detected && swingPhase > 0.8) // contact detection
                {
                    contactDetection[i] = true;
                }

                if (contactDetection[i] && gaitData->swingTimeRemain[i] < 0.1)
                {
                    jointsCmd->Kp.segment(3 * i, 3).setZero();
                    jointsCmd->Kd.segment(3 * i, 3).fill(0.5);
                    jointsCmd->qvel_des.segment(3 * i, 3).setZero();
                }
                else
                {
                    DMat J_inv = robotModelData->Je[i].middleCols(i * 3 + 6, 3).inverse();
                    Vec3 delta_q = J_inv * (tasksData->footTaskData[i].pos - estimatedState->footState[i].pos);
                    Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - tasksData->comTaskData.vWorld);
                    // Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - estimatedState->floatingBaseState.vWorld);
                    jointsCmd->qpos_des.segment(3 * i, 3) = estimatedState->jointsState.qpos.segment(3 * i, 3) + delta_q;
                    jointsCmd->qvel_des.segment(3 * i, 3) = qd_des;

                    if (gaitData->swingTimeRemain[i] / gaitData->swingTime[i] < 0.5)
                    {
                        jointsCmd->Kp.segment(3 * i, 3) << 30.0, 30.0, 30.0;
                        jointsCmd->Kd.segment(3 * i, 3) << 1.0, 1.0, 1.0;
                    }
                    else
                    {
                        jointsCmd->Kp.segment(3 * i, 3) << 30.0, 30.0, 30.0;
                        jointsCmd->Kd.segment(3 * i, 3) << 2.0, 2.0, 2.0;
                    }
                    ///leg swing partial WBC
                    Vec3 temp;
                    Mat3 Kp_sw; //Kp_swing_world
                    Mat3 Kd_sw; //Kd_swing_world

                    temp << 300, 300, 300;
                    Kp_sw = temp.asDiagonal();
                    temp << 40.0, 40.0, 40.0;
                    Kd_sw = temp.asDiagonal();

                    Vec3 a_des = Kp_sw * (tasksData->footTaskData[i].pos - estimatedState->footState[i].pos) + Kd_sw * (tasksData->footTaskData[i].vWorld - estimatedState->footState[i].vWorld) +
                                 tasksData->footTaskData[i].linAccWorld - robotModelData->Je_dot_q_dot[i];
                    Vec3 qddot_J = J_inv * (a_des - robotModelData->Je[i].leftCols(6) * qddot_f);

                    DMat D = robotModelData->M.block<3, 6>(6 + 3 * i, 6);
                    DMat Mj = robotModelData->M.block<3, 3>(6 + 3 * i, 6 + 3 * i);
                    jointsCmd->tau_ff.segment(3 * i, 3) =
                        D * qddot_f + Mj * qddot_J + robotModelData->bias.segment(6 + 3 * i, 3);
                    /* jointsCmd->Kp.segment(3 * i, 3).setZero();
                    jointsCmd->Kd.segment(3 * i, 3).setZero();
                    DMat Ji = robotModelData->Je[i].middleCols(i * 3 + 6, 3);
                    jointsCmd->tau_ff.segment(3 * i, 3) += Ji.transpose() * (1000.0 * (tasksData->footTaskData[i].pos - estimatedState->footState[i].pos - (tasksData->comTaskData.pos - estimatedState->floatingBaseState.pos)) + 40.0 * (tasksData->footTaskData[i].vWorld - estimatedState->footState[i].vWorld - (tasksData->comTaskData.vWorld - estimatedState->floatingBaseState.vWorld)) + (robotModelData->Je[i] * robotModelData->M.inverse() * robotModelData->Je[i].transpose()).inverse() * tasksData->footTaskData[i].linAccWorld); */
                }
            }
            else
            {
                jointsCmd->Kp.segment(3 * i, 3).setZero();
                jointsCmd->Kd.segment(3 * i, 3).fill(2.0);
                jointsCmd->qvel_des.segment(3 * i, 3).setZero();
            }
        }
        else // stance
        {
            contactDetection[i] = false;
            jointsCmd->Kp.segment(3 * i, 3) << 30, 30, 30;
            jointsCmd->Kd.segment(3 * i, 3) << 1.0, 1.0, 1.0;
            DMat J_inv = robotModelData->Je[i].middleCols(i * 3 + 6, 3).inverse();
            DMat Jf = robotModelData->Je[i].leftCols(6);

            Vec3 rotation_err = pin::log3(estimatedState->floatingBaseState.R_wb.transpose() *
                                          pin::rpy::rpyToMatrix(tasksData->comTaskData.rpy));
            Vec6 pose_err;
            pose_err << estimatedState->floatingBaseState.R_wb.transpose() *
                            (tasksData->comTaskData.pos - estimatedState->floatingBaseState.pos),
                rotation_err;
            Vec3 delta_q = -J_inv * Jf * (pose_err);
            Vec6 v_des;
            v_des << estimatedState->floatingBaseState.R_wb.transpose() * tasksData->comTaskData.vWorld,
                estimatedState->floatingBaseState.R_wb.transpose() * tasksData->comTaskData.omegaWorld;
            Vec3 qd_des = -J_inv * Jf * v_des;
            jointsCmd->qpos_des.segment(3 * i, 3) = estimatedState->jointsState.qpos.segment(3 * i, 3) + delta_q;
            jointsCmd->qvel_des.segment(3 * i, 3) = qd_des;

            // jointsCmd->Kp.segment(3 * i, 3) << 0, 0, 0;
            // jointsCmd->Kd.segment(3 * i, 3) << 0.1, 0.1, 0.1;
            // jointsCmd->qvel_des.segment(3 * i, 3).setZero();
        }
    }
    toRobot();
}

void LegController::toRobot()
{
    for (int i(0); i < 12; i++)
    {
        double tau = jointsCmd->tau_ff[i];
        if (abs(tau) > 40)
        {
            jointsCmd->tau_ff[i] = 40 * abs(tau) / tau;
        }
        else
        {
            jointsCmd->tau_ff[i] = tau;
        }
    }
}
/* ---------- swing test -------------*/

void LegController::swingTest(size_t leg)
{
    Vec4Int footID;
    footID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;
    jointsCmd->tau_ff.setZero();
    if (gaitData->swingTimeRemain[leg] > 0.)
    {
        if (first)
        {
            first = false;
            footSwingTrajectory.setInitialPosition(
                estimatedState->footState[leg].pos - estimatedState->floatingBaseState.pos);
            footSwingTrajectory.setFinalPosition(
                estimatedState->footState[leg].pos - estimatedState->floatingBaseState.pos);
            footSwingTrajectory.setHeight(0.05);
            std::cout << "s: "
                      << (estimatedState->footState[leg].pos - estimatedState->floatingBaseState.pos).transpose()
                      << std::endl;
        }

        double swingPhase = 1 - gaitData->swingTimeRemain[leg] / gaitData->swingTime[leg];
        footSwingTrajectory.computeSwingTrajectoryBezier(swingPhase, gaitData->swingTime[leg]);

        jointsCmd->Kp.segment(4 * leg, 4) << 30, 30, 30, 500;
        jointsCmd->Kd.segment(4 * leg, 4) << 1.0, 1.0, 1.0, 5;
        DMat J_inv = robotModelData->Je[leg].middleCols(leg * 4 + 6, 3).inverse();
        Vec3 delta_q = J_inv * (footSwingTrajectory.getPosition() -
                                (estimatedState->footState[0].pos -
                                 estimatedState->floatingBaseState.pos));
        Vec3 qd_des = J_inv * footSwingTrajectory.getVelocity();
        jointsCmd->qpos_des.segment(4 * leg, 3) = estimatedState->jointsState.qpos.segment(4 * leg, 3) + delta_q;
        jointsCmd->qvel_des.segment(4 * leg, 3) = qd_des;
        for (int i = leg * 4; i < leg * 4 + 4; i++)
        {
            double tau = robotModelData->generalizedGravity[i] +
                         jointsCmd->Kp[i] * (jointsCmd->qpos_des[i] - estimatedState->jointsState.qpos[i]) +
                         jointsCmd->Kd[i] *
                             (jointsCmd->qvel_des[i] - estimatedState->jointsState.qvel[i]);
            if (abs(tau) > 40)
            {
                jointsCmd->tau_ff[i] = 40 * abs(tau) / tau;
            }
            else
            {
                jointsCmd->tau_ff[i] = tau;
            }
        }
    }
    else
    {
        jointsCmd->tau_ff.setZero();
    }
}