//
// Created by nimapng on 3/30/21.
//

#include <iostream>
#include "DataSets.h"
#include "Controller.h"
#include "Estimator.h"
#include "GaitScheduler.h"
#include "Planner.h"
#include "Param.h"
#include "Configuration.h"
#include "Timer.h"
#include "MatFileHandler.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

TEST(LoadParameters, string_load) {
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    EXPECT_EQ(param.urdf.c_str(), std::string(THIS_COM) +"config/urdf/xiaotian.urdf");
}

TEST(LoadParameters, vector_load) {
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    std::vector<double> Q_pos_ref(3);
    Q_pos_ref[0] = 0.25;
    Q_pos_ref[1] = 0.25;
    Q_pos_ref[2] = 50;
    EXPECT_EQ(Q_pos_ref, param.Q_pos);
}

TEST(LoadParameters, value_load) {
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    double mu_ref = 0.4;
    int horizons_ref = 10;
    EXPECT_EQ(param.mu, mu_ref);
    EXPECT_EQ(param.horizons, horizons_ref);
}

TEST(RobotModel, link_name) {
    DataSets dataSets;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(),
                          dataSets.robotModelData.model);
    ASSERT_TRUE(dataSets.robotModelData.model.existBodyName(param.FL_WHEEL));
}

TEST(RobotModel, joint_name) {
    DataSets dataSets;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(),
                          dataSets.robotModelData.model);
    ASSERT_TRUE(dataSets.robotModelData.model.existJointName(param.FL_HIP));
}

TEST(RobotModel, LinkID) {
    DataSets dataSets;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(),
                          dataSets.robotModelData.model);
    ASSERT_TRUE(dataSets.robotModelData.model.getBodyId(param.FL_WHEEL) > 0);
    ASSERT_TRUE(dataSets.robotModelData.model.getBodyId(param.FR_WHEEL) > 0);
    ASSERT_TRUE(dataSets.robotModelData.model.getBodyId(param.HL_WHEEL) > 0);
    ASSERT_TRUE(dataSets.robotModelData.model.getBodyId(param.HR_WHEEL) > 0);
    ASSERT_TRUE(dataSets.robotModelData.model.getJointId(param.FL_WHEEL) > 0);
    ASSERT_TRUE(dataSets.robotModelData.model.getJointId(param.FR_WHEEL) > 0);
    ASSERT_TRUE(dataSets.robotModelData.model.getJointId(param.HL_WHEEL) > 0);
    ASSERT_TRUE(dataSets.robotModelData.model.getJointId(param.HR_WHEEL) > 0);
}

TEST(SharedMemory, MeasuredState) {
    //    TODO:
}

TEST(SharedMemory, JointsCmd) {
    //    TODO:
}

void setMeasuredState(MeasuredState &measuredState) {
    measuredState.cheatData.timeStep = 0.0;
    measuredState.cheatData.qpos.setZero();
    measuredState.cheatData.qpos[2] = 0.42;
    measuredState.cheatData.qpos[3] = 1;
    measuredState.cheatData.qvel.setZero();
}

TEST(Estimator, FloatingBaseState) {
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();

    for (int i = 0; i < 3; i++) {
        ASSERT_DOUBLE_EQ(measuredState.cheatData.qpos[i], estimatedState.floatingBaseState.pos[i]);
        ASSERT_DOUBLE_EQ(measuredState.cheatData.qvel[i], estimatedState.floatingBaseState.vBody[i]);
        ASSERT_DOUBLE_EQ(measuredState.cheatData.qvel[i + 3], estimatedState.floatingBaseState.omegaBody[i]);
        ASSERT_DOUBLE_EQ(measuredState.cheatData.qpos[i + 4], estimatedState.floatingBaseState.quat[i]);
    }
}

TEST(Estimator, JointsState) {
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();
    for (int i = 0; i < 16; i++) {
        ASSERT_DOUBLE_EQ(estimatedState.jointsState.qpos[i], measuredState.jointsState.qpos[i]);
        ASSERT_DOUBLE_EQ(estimatedState.jointsState.qvel[i], measuredState.jointsState.qvel[i]);
    }
    // TODO:
}

template<typename T>
bool almostEqual(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b,
                 T tol) {
    long x = T::RowsAtCompileTime;
    long y = T::ColsAtCompileTime;

    if (T::RowsAtCompileTime == Eigen::Dynamic ||
        T::ColsAtCompileTime == Eigen::Dynamic) {
        assert(a.rows() == b.rows());
        assert(a.cols() == b.cols());
        x = a.rows();
        y = a.cols();
    }

    for (long i = 0; i < x; i++) {
        for (long j = 0; j < y; j++) {
            T error = std::abs(a(i, j) - b(i, j));
            if (error >= tol)
                return false;
        }
    }
    return true;
}

TEST(Estimator, EndEffectorState) {
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();

    Vec23 qpos_pin;
    qpos_pin << estimatedState.floatingBaseState.pos,
            estimatedState.floatingBaseState.quat,
            estimatedState.jointsState.qpos;
    pin::framesForwardKinematics(robotModelData.model, robotModelData.data, qpos_pin);
    pin::SE3 FR;
    FR.rotation() = estimatedState.footState[1].R_wf;
    FR.translation() = estimatedState.footState[1].pos;
    EXPECT_TRUE(FR.isEqual(robotModelData.data.oMf[robotModelData.frwID]));
}

void setUserCmd(UserCmd &userCmd) {
    userCmd.gaitNum = size_t(GaitTypes::STAND);
    userCmd.vx_des = 0;
    userCmd.vy_des = 0;
    userCmd.yawd_des = 0;
}

/* TEST(GaitScheduler, GaitData_STANCE)
{
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();

    UserCmd userCmd;
    setUserCmd(userCmd);
    GaitData gaitData;
    GaitScheduler gait(&estimatedState,
                       &userCmd,
                       &gaitData,
                       &param);
    for (int j = 0; j < 200; j++)
    {
        gait.step();
        for (int i = 0; i < 4; i++)
        {
            ASSERT_TRUE(gaitData.swingTime[i] < 1e-16);
            // ASSERT_DOUBLE_EQ(gaitData.nextStanceTime[i], 0.54);
            // ASSERT_DOUBLE_EQ(gaitData.swingTimeRemain[i], 0.);
            // ASSERT_DOUBLE_EQ(gaitData.stanceTimeRemain[i], 0.54 - double(j * 0.002));
            ASSERT_TRUE(fabs(gaitData.nextStanceTime[i] - 0.54) < 1e-16);
            ASSERT_TRUE(fabs(gaitData.swingTimeRemain[i] - 0.)< 1e-16);
            ASSERT_TRUE(fabs(gaitData.stanceTimeRemain[i] - (0.54 - double(j * 0.002))) < 1e-8);
        }
    }
    // TODO:
}
 */
/* TEST(GaitScheduler, GaitData_TROTTING)
{
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();

    UserCmd userCmd;
    setUserCmd(userCmd);
    userCmd.gaitNum = size_t(GaitTypes::TROT_WALK);
    GaitData gaitData;
    GaitScheduler gait(&estimatedState,
                       &userCmd,
                       &gaitData,
                       &param);
    gait.step();
    for (int i = 0; i < 4; i++)
    {
        EXPECT_TRUE(gaitData.swingTime[i] == 0.27);
    }
    // TODO:
}
 */
/* TEST(Planner, FootPlanner)
{
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();

    UserCmd userCmd;
    setUserCmd(userCmd);
    GaitData gaitData;
    gaitData.swingTime.setZero();
    gaitData.nextStanceTime = 0.54 * Vec4::Ones();
    gaitData.swingTimeRemain.setZero();
    gaitData.stanceTimeRemain = 0.54 * Vec4::Ones();

    TasksData tasksData;
    Planner planner(&estimatedState,
                    &robotModelData,
                    &userCmd,
                    &gaitData,
                    &tasksData,
                    &param);
    planner.plan();
    
    // TODO:
}
 */
/* TEST(Planner, CoMPlanner)
{
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();

    UserCmd userCmd;
    setUserCmd(userCmd);
    GaitData gaitData;
    gaitData.swingTime.setZero();
    gaitData.nextStanceTime = 0.54 * Vec4::Ones();
    gaitData.swingTimeRemain.setZero();
    gaitData.stanceTimeRemain = 0.54 * Vec4::Ones();

    TasksData tasksData;
    Planner planner(&estimatedState,
                    &robotModelData,
                    &userCmd,
                    &gaitData,
                    &tasksData,
                    &param);
    planner.plan();
    for (int i = 0; i < 3; i++)
    {
        ASSERT_TRUE(abs(tasksData.comTaskData.pos[i]) < 1e-16);
        ASSERT_TRUE(abs(tasksData.comTaskData.rpy[i]) < 1e-16);
        ASSERT_TRUE(abs(tasksData.comTaskData.vWorld[i]) < 1e-16);
        ASSERT_TRUE(abs(tasksData.comTaskData.omegaWorld[i]) < 1e-16);
    }
    double mass_total = 0;
    for (auto &mass : robotModelData.data.mass)
    {
        mass_total += mass;
    }
    double frc_z = tasksData.forcesTaskData.forces_ref[2] + tasksData.forcesTaskData.forces_ref[5] + tasksData.forcesTaskData.forces_ref[8] + tasksData.forcesTaskData.forces_ref[11];
    ASSERT_TRUE(abs(frc_z - mass_total * 9.81) < 1e-3) << "frc_z: " << frc_z << ", gravity: " << mass_total * 9.81 << "\n frc_des: " << tasksData.forcesTaskData.forces_ref.transpose();

    // TODO:
} */

/* TEST(Planner, CoMPlanner) {
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");

    ForcesTaskData forcesTaskData;
    CoMPlannerMPC comPlanner(nullptr, nullptr, nullptr, nullptr, &forcesTaskData, nullptr, &param);

    DMat rpyList(3, 5), pList(3, 5), omegaList(3, 5), vList(3, 5), rList(12, 5);
    DMatInt mpcTableList(4 * param.horizons, 5);
    DMat trajAll(12 * param.horizons, 5), forcesRefList(12, 5);

    readMat(THIS_COM"test/TestData.mat", "rpy", rpyList);
    readMat(THIS_COM"/test/TestData.mat", "pos", pList);
    readMat(THIS_COM"/test/TestData.mat", "omegaWorld", omegaList);
    readMat(THIS_COM"/test/TestData.mat", "vWorld", vList);
    readMat(THIS_COM"/test/TestData.mat", "r", rList);
    readMat(THIS_COM"/test/TestData.mat", "mpcTable", mpcTableList);
    readMat(THIS_COM"/test/TestData.mat", "trajAll", trajAll);
    readMat(THIS_COM"/test/TestData.mat", "force", forcesRefList);

    std::cout << "rpyList: \n" << rpyList << "\n";
    std::cout << "pList: \n" << pList << "\n";
    std::cout << "omegaList: \n" << omegaList << "\n";
    std::cout << "vList: \n" << vList << "\n";
    std::cout << "rList: \n" << rList << "\n";
    std::cout << "mpcTableList: \n" << mpcTableList << "\n";
    std::cout << "trajALl: \n" << trajAll << "\n";

    for (int i(0); i < 5; i++) {
        comPlanner.ctModel(rpyList.col(i), rList.col(i));
        comPlanner.dtModel();
        comPlanner.setInitialState(rpyList.col(i), pList.col(i), omegaList.col(i), vList.col(i));
        comPlanner.setReferenceTraj(trajAll.col(i));
        comPlanner.setMPCTable(mpcTableList.col(i).cast<int>());
        comPlanner.solveQP();

        auto error = forcesRefList.col(i) - forcesTaskData.forces_ref;
        std::cout << "Our forces: " << forcesTaskData.forces_ref.transpose() << "\n";
        std::cout << "MIT forces: " << forcesRefList.col(i).transpose() << "\n";

        std::cout << "force error: " << error.transpose() << "\n\n";
        ASSERT_TRUE(error.norm() < 1e-5);
    }
}
 */
/* TEST(Controller, WBC)
{
    RobotModelData robotModelData;
    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(), robotModelData.model);
    robotModelData.data = pin::Data(robotModelData.model);
    robotModelData.flwID = robotModelData.model.getBodyId(param.FL_WHEEL);
    robotModelData.frwID = robotModelData.model.getBodyId(param.FR_WHEEL);
    robotModelData.hlwID = robotModelData.model.getBodyId(param.HL_WHEEL);
    robotModelData.hrwID = robotModelData.model.getBodyId(param.HR_WHEEL);
    robotModelData.flhID = robotModelData.model.getJointId(param.FL_WHEEL);
    robotModelData.frhID = robotModelData.model.getJointId(param.FR_WHEEL);
    robotModelData.hlhID = robotModelData.model.getJointId(param.HL_WHEEL);
    robotModelData.hrhID = robotModelData.model.getJointId(param.HR_WHEEL);

    MeasuredState measuredState;
    EstimatedState estimatedState;
    Estimator estimator(&measuredState,
                        &robotModelData,
                        &estimatedState,
                        &param);
    setMeasuredState(measuredState);
    estimator.estimate();

    UserCmd userCmd;
    GaitData gaitData;
    GaitScheduler gait(&estimatedState,
                       &userCmd,
                       &gaitData,
                       &param);
    gait.step();

    TasksData tasksData;
    Planner planner(&estimatedState,
                    &robotModelData,
                    &userCmd,
                    &gaitData,
                    &tasksData,
                    &param);
    planner.plan();

    JointsCmd jointsCmd;
    Controller controller(&estimatedState,
                          &robotModelData,
                          &tasksData,
                          &gaitData,
                          &jointsCmd,
                          &param);
    // TODO:
}
 */