//
// Created by nimapng on 12/13/21.
//

#include "TSC.h"

TSC_IMPL::TSC_IMPL(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
                   const TasksData *tasksData,
                   const GaitData *gaitData, JointsCmd *jointsCmd, const UserParameterHandler *param) : estimatedState(estimatedState),
                                                                                                        robotModelData(robotModelData),
                                                                                                        tasksData(tasksData),
                                                                                                        gaitData(gaitData),
                                                                                                        jointsCmd(jointsCmd),
                                                                                                        param(param),
                                                                                                        _robot(param->urdf, false),
                                                                                                        mt_waist(_robot, param->BASE),
                                                                                                        forceTask(_robot, "ForceTask"),
                                                                                                        rt(_robot, "RegularizationTask"),
                                                                                                        angularMomentumTask(_robot, "AngularMomentumTask"),
                                                                                                        fl(_robot, param->FL_WHEEL),
                                                                                                        fr(_robot, param->FR_WHEEL),
                                                                                                        rl(_robot, param->HL_WHEEL),
                                                                                                        rr(_robot, param->HR_WHEEL),
                                                                                                        cpcstr(_robot, "cpcstr"),
                                                                                                        cfcstr(_robot, "cfcstr"),
                                                                                                        actuatorLimit(_robot, "ActuatorLimit"),
                                                                                                        tsc(_robot)

{

    mt_waist.Kp().diagonal() << 300, 300, 400, 400, 400, 400;
    mt_waist.Kd() = 2 * mt_waist.Kp().cwiseSqrt();
    mt_waist.weightMatrix().diagonal() << 1000, 1000, 1000, 500, 500, 500;

    forceTask.weightMatrix().diagonal().fill(10);

    rt.qaccWeight().diagonal().fill(1e-5);
    rt.forceWeight().diagonal().fill(1e-8);

    angularMomentumTask.weightMatrix().diagonal().fill(1);
    angularMomentumTask.Kp().diagonal().fill(20);
    angularMomentumTask.ref().setZero();
    angularMomentumTask.ref_dot().setZero();

    fl.Kp().diagonal() << 50, 50, 60, 0, 0, 0;
    fl.Kd() = 1.5 * fl.Kp().cwiseSqrt();
    fl.weightMatrix().diagonal() << 1000, 1000, 1000, 0, 0, 0;

    fr.Kp().diagonal() << 50, 50, 60, 0, 0, 0;
    fr.Kd() = 1.5 * fr.Kp().cwiseSqrt();
    fr.weightMatrix().diagonal() << 1000, 1000, 1000, 0, 0, 0;

    rl.Kp().diagonal() << 50, 50, 60, 0, 0, 0;
    rl.Kd() = 1.5 * rl.Kp().cwiseSqrt();
    rl.weightMatrix().diagonal() << 1000, 1000, 1000, 0, 0, 0;

    rr.Kp().diagonal() << 50, 50, 60, 0, 0, 0;
    rr.Kd() = 1.5 * rr.Kp().cwiseSqrt();
    rr.weightMatrix().diagonal() << 1000, 1000, 1000, 0, 0, 0;

    cfcstr.mu() = 0.6;
    cfcstr.max() = 200;

    tsc.addTask(&mt_waist);
    //    tsc.addTask(angularMomentumTask);
    tsc.addTask(&forceTask);
    tsc.addLinearConstraint(&cpcstr);
    tsc.addLinearConstraint(&cfcstr);
    tsc.addTask(&fl);
    tsc.addTask(&fr);
    tsc.addTask(&rr);
    tsc.addTask(&rl);
    tsc.addTask(&rt);
    tsc.addLinearConstraint(&actuatorLimit);

    contact_virtual_link.emplace_back("FL_foot");
    contact_virtual_link.emplace_back("FR_foot");
    contact_virtual_link.emplace_back("RL_foot");
    contact_virtual_link.emplace_back("RR_foot");
    _robot.setContactVirtualLink(contact_virtual_link);
}

void TSC_IMPL::run()
{
    Vec19 qpos_pin;
    qpos_pin << estimatedState->floatingBaseState.pos, estimatedState->floatingBaseState.quat.tail(3), estimatedState->floatingBaseState.quat(0), estimatedState->jointsState.qpos;
    Vec18 qvel_pin;
    qvel_pin << estimatedState->floatingBaseState.vBody, estimatedState->floatingBaseState.omegaBody,
        estimatedState->jointsState.qvel;
    _robot.update(qpos_pin, qvel_pin);

    VecXi contactState(4);
    for (int i(0); i < 4; i++)
    {
        if (gaitData->stanceTimeRemain[i] > 0.)
            contactState[i] = 1;
        else
            contactState[i] = 0;
    }

    auto fl_pose = _robot.frame_pose(fl.name());
    fl.SE3Ref().translation() = tasksData->footTaskData[0].pos;
    fl.spatialVelRef().head(3) = fl_pose.rotation().transpose() * tasksData->footTaskData[0].vWorld;
    fl.spatialAccRef().head(3) = fl_pose.rotation().transpose() * (tasksData->footTaskData[0].linAccWorld - estimatedState->footState[0].omegaWorld.cross(estimatedState->footState[0].vWorld));
    if (!tsc.existTask(fl.name()))
    {
        tsc.addTask(&fl);
    }

    auto fr_pose = _robot.frame_pose(fr.name());
    fr.SE3Ref().translation() = tasksData->footTaskData[1].pos;
    fr.spatialVelRef().head(3) = fr_pose.rotation().transpose() * tasksData->footTaskData[1].vWorld;
    fr.spatialAccRef().head(3) = fr_pose.rotation().transpose() * (tasksData->footTaskData[1].linAccWorld - estimatedState->footState[1].omegaWorld.cross(estimatedState->footState[1].vWorld));
    if (!tsc.existTask(fr.name()))
    {
        tsc.addTask(&fr);
    }

    auto rl_pose = _robot.frame_pose(rl.name());
    rl.SE3Ref().translation() = tasksData->footTaskData[2].pos;
    rl.spatialVelRef().head(3) = rl_pose.rotation().transpose() * tasksData->footTaskData[2].vWorld;
    rl.spatialAccRef().head(3) = rl_pose.rotation().transpose() * (tasksData->footTaskData[2].linAccWorld - estimatedState->footState[2].omegaWorld.cross(estimatedState->footState[2].vWorld));
    if (!tsc.existTask(rl.name()))
    {
        tsc.addTask(&rl);
    }

    auto rr_pose = _robot.frame_pose(rr.name());
    rr.SE3Ref().translation() = tasksData->footTaskData[3].pos;
    rr.spatialVelRef().head(3) = rr_pose.rotation().transpose() * tasksData->footTaskData[3].vWorld;
    rr.spatialAccRef().head(3) = rr_pose.rotation().transpose() * (tasksData->footTaskData[3].linAccWorld - estimatedState->footState[3].omegaWorld.cross(estimatedState->footState[3].vWorld));
    if (!tsc.existTask(rr.name()))
    {
        tsc.addTask(&rr);
    }

    auto base_frame = _robot.frame_pose(mt_waist.name());
    auto base_vel = _robot.frame_6dVel_local(mt_waist.name());

    mt_waist.SE3Ref().translation() = tasksData->comTaskData.pos;
    mt_waist.spatialVelRef().head(3) = base_frame.rotation().transpose() * tasksData->comTaskData.vWorld;
    mt_waist.spatialVelRef().tail(3) = base_frame.rotation().transpose() * tasksData->comTaskData.omegaWorld;
    mt_waist.spatialAccRef().head(3) = base_frame.rotation().transpose() * tasksData->comTaskData.linAccWorld -
                                       base_vel.angular().cross(base_vel.linear());
    mt_waist.spatialAccRef().tail(3) = base_frame.rotation().transpose() * tasksData->comTaskData.angAccWorld;
    mt_waist.SE3Ref().rotation() = pin::rpy::rpyToMatrix(tasksData->comTaskData.rpy);
    forceTask.setForceRef(tasksData->forcesTaskData.forces_ref);

    _robot.compute(contactState);

    tsc.solve();
    jointsCmd->tau_ff = tsc.getOptimalTorque().tail(12);

//    cout << "force ref: " << tasksData->forcesTaskData.forces_ref.transpose() << endl;
//    cout << "force tsc: " << tsc.getOptimalContactForce().transpose() << endl;

    ik();
}

void TSC_IMPL::ik()
{
    Matrix<double, 12, 18> Jc_all;
    for (int i = 0; i < 4; i++)
    {
        Jc_all.block(3 * i, 0, 3, 18) = robotModelData->Je[i]; // TODO Jc
    }

    Vec4Int footID;
    footID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;

    DMat Mf = robotModelData->M.topLeftCorner<6, 6>();
    Vec6 qddot_f;
    qddot_f << estimatedState->floatingBaseState.aBody, 0, 0, 0;
    for (int i = 0; i < 4; i++)
    {
        if (gaitData->swingTimeRemain[i] > 0.) // swing
        {
            jointsCmd->Kp.segment(3 * i, 3).fill(30.0);
            jointsCmd->Kd.segment(3 * i, 3).fill(2.0);
            DMat J_inv = robotModelData->Je[i].middleCols(i * 3 + 6, 3).inverse();
            Vec3 delta_q = J_inv * (tasksData->footTaskData[i].pos - estimatedState->footState[i].pos);
            // Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - tasksData->comTaskData.vWorld);
            Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - estimatedState->floatingBaseState.vWorld);
            jointsCmd->qpos_des.segment(3 * i, 3) = estimatedState->jointsState.qpos.segment(3 * i, 3) + delta_q;
            jointsCmd->qvel_des.segment(3 * i, 3) = qd_des;
        }
        else // stance
        {
//            jointsCmd->Kp.segment(3 * i, 3).setZero();
//            jointsCmd->Kd.segment(3 * i, 3).setZero();
//            // DMat J_inv = robotModelData->Je[i].middleCols(i * 3 + 6, 3).inverse();
//            // Vec3 delta_q = J_inv * (estimatedState->floatingBaseState.pos - tasksData->comTaskData.pos);
//            // Vec3 qd_des = J_inv * (estimatedState->floatingBaseState.vWorld - tasksData->comTaskData.vWorld);
//            // jointsCmd->qpos_des.segment(3 * i, 3) = estimatedState->jointsState.qpos.segment(3 * i, 3) + delta_q;
//            // jointsCmd->qvel_des.segment(3 * i, 3) = qd_des;
//            jointsCmd->qpos_des.segment(3 * i, 3) = estimatedState->jointsState.qpos.segment(3 * i, 3);
//            jointsCmd->qvel_des.segment(3 * i, 3).setZero();

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
        }
    }
    // for (int i(0); i < 12; i++)
    // {
    //     double tau = jointsCmd->tau_ff[i] +
    //                  jointsCmd->Kp[i] * (jointsCmd->qpos_des[i] - estimatedState->jointsState.qpos[i]) +
    //                  jointsCmd->Kd[i] *
    //                      (jointsCmd->qvel_des[i] - estimatedState->jointsState.qvel[i]);
    //     if (abs(tau) > 40)
    //     {
    //         jointsCmd->tau_ff[i] = 40 * abs(tau) / tau;
    //     }
    //     else
    //     {
    //         jointsCmd->tau_ff[i] = tau;
    //     }
    // }
}
