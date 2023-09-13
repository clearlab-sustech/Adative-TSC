//
// Created by wenchun on 3/22/21.
//

#include "CoMPlannerMPC.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "orientation_tools.h"

#define BIG_NUMBER 5e10
#define PRINT_PROBLEM_DATA

CoMPlannerMPC::CoMPlannerMPC(const EstimatedState *estimatedState,
                             const RobotModelData *robotModelData,
                             const UserCmd *userCmd,
                             const GaitData *gaitData,
                             FootTaskData *footTaskData,
                             ForcesTaskData *forcesTaskData,
                             CoMTaskData *comTaskData,
                             const UserParameterHandler *param) : estimatedState(estimatedState),
                                                                  robotModelData(robotModelData),
                                                                  userCmd(userCmd),
                                                                  gaitData(gaitData),
                                                                  footTaskData(footTaskData),
                                                                  forcesTaskData(forcesTaskData),
                                                                  comTaskData(comTaskData),
                                                                  param(param),
                                                                  srgbMpc(param->horizons, param->dtMPC),
                                                                  poplarMpc(param->horizons, param->dtMPC),
                                                                  _iter(0),
                                                                  iter_init(0)
{
    rpInt.setZero();
    rpCom.setZero();
    trajInit.setZero();

    x0.setZero();

    X_d.resize(13 * param->horizons, Eigen::NoChange);
    U_b.resize(20 * param->horizons, Eigen::NoChange);
    contactTable.resize(4 * param->horizons, Eigen::NoChange);
    fmat.resize(20 * param->horizons, 12 * param->horizons);

    X_d.setZero();
    U_b.setZero();
    contactTable.setZero();
    fmat.setZero();

    q_soln.resize(12 * param->horizons);
    q_soln.setZero();
    // TODO add inertia
    /*Vec3 Id;
    Id << .07f, 0.26f, 0.242f; // miniCheetah*/
    /*Ibody << 0.353682, -0.000624863, -0.0313391,
            -0.000624863, 0.813405, -0.000940105,
            -0.0313391, -0.000940105, 0.863894;
    */
    /*Ibody.setZero();
    Ibody.diagonal() << 3 *  Id;*/
    /*
    mass = 15.8198;*/
    Ibody << 0.155919, -0.000957476, -0.0047469,
        -0.000957476, 0.831677, 0.000276135,
        -0.0047469, 0.000276135, 0.942546;
    // Ibody << 0.271079, -0.000614604, -0.00564138,
    //     -0.000614604, 0.914908, -0.000527302,
    //     -0.00564138, -0.000527302, 0.918251;
    /*Ibody.setZero();
    Ibody.diagonal() << 3 *  Id;*/
    mass = 20.03226;

    SRGB_MPC::Vec Qx(12);
    Qx.segment(0, 3) << param->Q_rpy[0], param->Q_rpy[1], param->Q_rpy[2];
    Qx.segment(3, 3) << param->Q_pos[0], param->Q_pos[1], param->Q_pos[2];
    Qx.segment(6, 3) << param->Q_omega[0], param->Q_omega[1], param->Q_omega[2];
    Qx.segment(9, 3) << param->Q_vel[0], param->Q_vel[1], param->Q_vel[2];
    SRGB_MPC::Vec Qf(3);
    Qf.fill(4e-5);
    srgbMpc.setWeight(Qx, Qf);
    srgbMpc.setMassAndInertia(mass, Ibody);
    srgbMpc.setMaxForce(200);
    srgbMpc.setFrictionCoefficient(param->mu);

    poplarMpc.setWeight(Qx, Qf);
    poplarMpc.setMassAndInertia(mass, Ibody);
    poplarMpc.setMaxForce(200);
    poplarMpc.setDesiredFootZPos(0);

    _ext_wrench.setZero();
    vBodyDes.setZero();
    bodyHeight = param->body_height;

    for (int i(0); i < 4; i++)
    {
        pHipBody[i] = Vec3((i < 2) ? 0.234701 : -0.2336, (i == 0 || i == 2) ? 0.13 : -0.13, 0);
    }
}

void CoMPlannerMPC::setExternalWrench(Vec6 wrench)
{
    _ext_wrench = wrench;
}

void CoMPlannerMPC::plan()
{
    x0 << estimatedState->floatingBaseState.rpy, estimatedState->floatingBaseState.pos,
        estimatedState->floatingBaseState.omegaWorld, estimatedState->floatingBaseState.vWorld, -9.81;
    srgbMpc.setCurrentState(x0);
    poplarMpc.setCurrentState(x0);

    generateRefTraj();
    srgbMpc.setDesiredDiscreteTrajectory(X_d);
    poplarMpc.setDesiredDiscreteTrajectory(X_d);

    generateContactTable();
    Matrix<int, -1, -1> contactTable_4h(4, param->horizons);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < param->horizons; j++)
        {
            contactTable_4h(i, j) = contactTable(4 * j + i);
        }
    }
    srgbMpc.setContactTable(contactTable_4h);
    poplarMpc.setContactTable(contactTable_4h);

    // cout << "------------contact table--------------\n"
    //      << contactTable_4h << std::endl;

    srgbMpc.setExternalWrench(_ext_wrench);
    poplarMpc.setExternalWrench(_ext_wrench);

    Vec3 r;
    vector<Vec3> cp;
    vector<Vec3> footholds;
    for (int leg(0); leg < 4; leg++)
    {
        if (gaitData->swingTimeRemain[leg] > 0 && _iter > 0)
        {
            r = footTaskData[leg].nextContactPos;
        }
        else
        {
            r = estimatedState->footState[leg].pos;
        }
        footholds.push_back(estimatedState->footState[leg].pos);
        std::cout << "leg " << leg << ": " << estimatedState->footState[leg].pos.transpose() << "\n";
        cp.push_back(r);
    }
    Vec12 fh_n;
    for (int i = 0; i < 4; i++)
    {
        fh_n.segment(i * 3, 3) = estimatedState->floatingBaseState.pos + estimatedState->floatingBaseState.R_wb *
                                                                             (pHipBody[i] + estimatedState->floatingBaseState.vBody * gaitData->swingTimeRemain[i]);
        fh_n(i * 3 + 2) = 0.0;
    }
    poplarMpc.setNorminalFootholds(fh_n);

    srgbMpc.setContactPointPos(cp);
    poplarMpc.setContactPointPos(cp);
    poplarMpc.setCurrentFootholds(footholds);
    poplarMpc.setSwingTimeRemain(gaitData->swingTimeRemain);
    poplarMpc.setStanceTimeRemain(gaitData->stanceTimeRemain);

    // srgbMpc.solve(0.0);
    poplarMpc.solve(0.0);
    forcesTaskData->forces_ref = poplarMpc.getCurrentDesiredContactForce();
    for (int i = 0; i < 4; i++)
    {
        if (gaitData->late_contact[i])
        {
            forcesTaskData->forces_ref.segment(3 * i, 3) << 0, 0, mass * 9.81 / 4;
        }
    }
    // std::cout << "MPC Forces: \n"
    //   << forcesTaskData->forces_ref.transpose() << "\n";
    //    std::cout << "PoplarMPC Forces: \n"
    //              << poplarMpc.getCurrentDesiredContactForce().transpose() << "\n";
    //    updateTaskData();
    Vec12 fh_des = poplarMpc.getCurrentDesiredFootholds();
    for (int i = 0; i < 4; i++)
    {
        /* if (gaitData->swingTimeRemain[i] > 0)
        {
            footTaskData[i].nextContactPos = fh_des.segment(3 * i, 3);
        } */
        footTaskData[i].nextContactPos = fh_des.segment(3 * i, 3);
        std::cout << "leg des " << i << ": " << footTaskData[i].nextContactPos.transpose() << "\n";
    }
    _ext_wrench.setZero();
    _iter++;
}

void CoMPlannerMPC::planImediate()
{
    x0 << estimatedState->floatingBaseState.rpy, estimatedState->floatingBaseState.pos,
        estimatedState->floatingBaseState.omegaWorld, estimatedState->floatingBaseState.vWorld, -9.81;
    srgbMpc.solveImediate(x0);
    forcesTaskData->forces_ref = srgbMpc.getCurrentDesiredContactForce();
    // std::cout << "MPC Forces imediate: \n"
    //           << forcesTaskData->forces_ref.transpose() << "\n";
    //    updateTaskData();
}

void CoMPlannerMPC::generateHighLevelRef()
{

    for (int i = 0; i < 4; i++)
    {
        if (gaitData->swingTimeRemain[i] <= 0.) // stance
        {
            contactPoint[i] = estimatedState->footState[i].pos;
        }
    }
    // updatePostureRef();
    // Vec3 _vBodyDes = 0.8 * vBodyDes + 0.2 * Vec3(userCmd->vx_des, userCmd->vy_des, 0);
    Vec3 _vBodyDes = 0 * vBodyDes + 1 * comTaskData->footPlannedVBody;
    Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * _vBodyDes;

    double foot_height = 0;
    double ffx = gaitData->stanceTimeRemain[0] > 0 ? estimatedState->footState[0].pos.x()
                                                   : estimatedState->footState[1].pos.x();
    double fhx = gaitData->stanceTimeRemain[2] > 0 ? estimatedState->footState[2].pos.x()
                                                   : estimatedState->footState[3].pos.x();
    double ffz = gaitData->stanceTimeRemain[0] > 0 ? estimatedState->footState[0].pos.z()
                                                   : estimatedState->footState[1].pos.z();
    double fhz = gaitData->stanceTimeRemain[2] > 0 ? estimatedState->footState[2].pos.z()
                                                   : estimatedState->footState[3].pos.z();

    // std::cout << "pitchDes = " << pitchDes << std::endl;
    // std::cout << "pitchDes111 = " << 0.99 * pitchDes - 0.01 * atan((ffz - fhz) / (ffx - fhx)) << std::endl;
    pitchDes = 0.99 * pitchDes - 0.01 * atan((ffz - fhz) / (ffx - fhx));
    // pitchDes = 0.99 * pitchDes + 0.01 * comTaskData->pitchDes; /// Todo: given by footplanner
    // pitchDes = 0;

    for (int i = 0; i < 4; i++)
    {
        if (gaitData->stanceTimeRemain[i] > 0 || _iter < 5)
        {
            foot_height += estimatedState->footState[i].pos.z();
        }
        else
        {
            foot_height += footTaskData[i].nextContactPos.z();
        }
    }
    foot_height = foot_height / 4.0;
    poplarMpc.setDesiredFootZPos(foot_height);
    bodyHeight = 0.01 * (foot_height + param->body_height) + 0.99 * bodyHeight;
    // bodyHeight = param->body_height; //Todo: given by footplanner

    if (iter_init < 20)
    {
        xDes = estimatedState->floatingBaseState.pos[0];
        yDes = estimatedState->floatingBaseState.pos[1];
        yawDes = estimatedState->floatingBaseState.rpy[2];
        iter_init++;
    }
    else
    {
        xDes += vWorldDes[0] * param->dt;
        yDes += vWorldDes[1] * param->dt;
        // yawDes = estimatedState->floatingBaseState.rpy[2] + userCmd->yawd_des * param->dt;
        // yawDes = estimatedState->floatingBaseState.rpy[2] + userCmd->yawd_des * param->dtMPC;
        yawDes += userCmd->yawd_des * param->dt;
    }
    rpInt[0] += param->dt * (rollDes - estimatedState->floatingBaseState.rpy[0]);
    rpInt[1] += param->dt * (pitchDes - estimatedState->floatingBaseState.rpy[1]);
    rpInt[2] += param->dt * (yawDes - estimatedState->floatingBaseState.rpy[2]);
    rpCom[0] = rollDes + rpInt[0];
    rpCom[1] = pitchDes + rpInt[1];
    rpCom[2] = yawDes + rpInt[2];
}

void CoMPlannerMPC::generateContactTable()
{
    Vec4Int stTimeRemainInt = (gaitData->stanceTimeRemain / param->dtMPC).cast<int>();
    Vec4Int swTimeRemainInt = (gaitData->swingTimeRemain / param->dtMPC).cast<int>();
    Vec4Int swTimeInt = (gaitData->swingTime / param->dtMPC).cast<int>();
    Vec4Int stTimeInt = (gaitData->nextStanceTime / param->dtMPC).cast<int>();

    for (int i(0); i < param->horizons; i++)
    {
        for (int j(0); j < 4; j++)
        {
            if (stTimeRemainInt[j] > 0)
            { // leg is in stance
                if (i < stTimeRemainInt[j])
                    contactTable[i * 4 + j] = 1;
                else if (i >= stTimeRemainInt[j] &&
                         i < (swTimeInt[j] + stTimeRemainInt[j]))
                    contactTable[i * 4 + j] = 0;
                else if (i >= (swTimeInt[j] + stTimeRemainInt[j]) &&
                         i < (swTimeInt[j] + stTimeRemainInt[j] + stTimeInt[j]))
                    contactTable[i * 4 + j] = 1;
                else
                    printf("Predicted horizons exceed one gait cycle!\n");
            }
            else
            { // leg in swing
                if (i < swTimeRemainInt[j])
                    contactTable[i * 4 + j] = 0;
                else if (i >= swTimeRemainInt[j] &&
                         i < (swTimeRemainInt[j] + stTimeInt[j]))
                    contactTable[i * 4 + j] = 1;
                else if (i >= (swTimeRemainInt[j] + stTimeInt[j]) &&
                         i < (swTimeRemainInt[j] + stTimeInt[j] + swTimeInt[j]))
                    contactTable[i * 4 + j] = 0;
                else
                    printf("Predicted horizons exceed one gait cycle!\n");
            }
        }
    }
}

void CoMPlannerMPC::generateRefTraj()
{
    // vBodyDes = 0.8 * vBodyDes + 0.2 * Vec3(userCmd->vx_des, userCmd->vy_des, 0);
    vBodyDes = 0 * vBodyDes + 1 * comTaskData->footPlannedVBody;
    Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * vBodyDes;

    Vec3 pos = estimatedState->floatingBaseState.pos;
    Vec3 rpy = estimatedState->floatingBaseState.rpy;

    if (xDes - pos[0] > maxPosError)
        xDes = pos[0] + maxPosError;
    if (pos[0] - xDes > maxPosError)
        xDes = pos[0] - maxPosError;

    if (yDes - pos[1] > maxPosError)
        yDes = pos[1] + maxPosError;
    if (pos[1] - yDes > maxPosError)
        yDes = pos[1] - maxPosError;

    if (yawDes - rpy[2] > 0.5)
        yawDes = rpy[2] + 0.5;
    if (rpy[2] - yawDes > 0.5)
        yawDes = rpy[2] - 0.5;

#ifdef USE_MPC_STANCE
    double _bodyHeight_slow;
    if (_iter < 200)
    {
        _bodyHeight_slow = 0.05 + 0.30 / 200 * _iter;
    }
    else
    {
        _bodyHeight_slow = bodyHeight;
    }
    trajInit << rpCom[0], rpCom[1], yawDes,
        xDes, yDes, _bodyHeight_slow,
        0.0, 0.0, yawdDes,
        vWorldDes;
#else
    trajInit << rpCom[0], rpCom[1], rpCom[2],
        xDes, yDes, bodyHeight,
        0.0, 0.0, yawdDes,
        vWorldDes;
#endif

    X_d.head(12) = trajInit;
    //    std::cout << "x_0 " << x0.transpose() << std::endl;
    for (int i = 1; i < param->horizons; i++)
    {
        for (int j = 0; j < 12; j++)
            X_d[13 * i + j] = trajInit[j];
        X_d[13 * i + 2] = trajInit[2] + i * yawdDes * param->dtMPC;
        vWorldDes = pin::rpy::rpyToMatrix(X_d.head(3)) * vBodyDes;
        X_d.segment(13 * i + 3, 3) = trajInit.segment(3, 3) +
                                     i * vWorldDes * param->dtMPC;
        //        std::cout << "x_d " << i << ": " << X_d.segment(13 * i, 13).transpose() << std::endl;
    }
}

void CoMPlannerMPC::updateTaskData()
{
    // getchar();
    // Matrix<double,13,1> xnow;
    // xnow << estimatedState->floatingBaseState.rpy,
    //         estimatedState->floatingBaseState.pos,
    //         estimatedState->floatingBaseState.omegaWorld,
    //         estimatedState->floatingBaseState.vWorld, -9.81;
    // auto xopt = srgbMpc.getDiscreteOptimizedTrajectory();
    // auto xdd = srgbMpc.getXDot();

    auto xopt = poplarMpc.getDiscreteOptimizedTrajectory();
    auto xdd = poplarMpc.getXDot();

    comTaskData->pos = xopt.segment(3, 3);
    comTaskData->vWorld = xopt.segment(9, 3);
    comTaskData->linAccWorld = xdd.segment(9, 3);
    comTaskData->angAccWorld = xdd.segment(6, 3);
    comTaskData->rpy = xopt.head(3);
    comTaskData->omegaWorld = xopt.segment(6, 3);
}

/* for test */
void CoMPlannerMPC::setInitialState(const Vec3 &rpy, const Vec3 &p, const Vec3 &omega, const Vec3 &v)
{
    x0 << rpy, p, omega, v, -9.8;
    // std::cout << "x0: " << x0.transpose() << "\n";
}

void CoMPlannerMPC::setMPCTable(const Matrix<int, Dynamic, 1> &mpcTable)
{
    contactTable = mpcTable;
}

void CoMPlannerMPC::setReferenceTraj(const DVec &Xref)
{
    for (int i(0); i < param->horizons; i++)
        for (int j(0); j < 12; j++)
            X_d(13 * i + j, 0) = Xref(12 * i + j, 0);
}

void CoMPlannerMPC::updatePostureRef()
{
    Vec3 a;
    Vec4 Pz;
    Mat43 W;
    a.setZero();
    W.block<4, 1>(0, 0) << 1, 1, 1, 1;
    for (int i = 0; i < 4; i++)
    {
        // W.block<1,2>(i,1) << estimatedState->footState[i].contactPoint[0], estimatedState->footState[i].contactPoint[1];
        // Pz[i] = estimatedState->footState[i].contactPoint[2];
        W.block<1, 2>(i, 1) << contactPoint[i][0], contactPoint[i][1];
        Pz[i] = contactPoint[i][2];
    }
    a = (W.transpose() * W).inverse() * W.transpose() * Pz;
    double pitch, roll;
    Vec3 z1, z2;
    double yaw = estimatedState->floatingBaseState.rpy[2];
    // double yaw = userCmd->yawd_des;
    z1 << 1, cos(yaw), sin(yaw);
    z2 << 1, sin(yaw), -cos(yaw);
    pitchDes = -atan2(a.transpose() * z1 - a[0], 1);
    rollDes = -atan2(a.transpose() * z2 - a[0], 1);
    // std::cout << "a = " << a.transpose() << std::endl;
    // for (int i = 0; i < 4; i++)
    // {
    //     std::cout << "Contactpoint" <<i <<"  = " << contactPoint[i].transpose() << std::endl;
    // }

    // std::cout << "pitch = " << pitchDes << std::endl;
    // std::cout << "roll = " << roll << std::endl;
    // std::cout << "yaw = " << yaw << std::endl;
}