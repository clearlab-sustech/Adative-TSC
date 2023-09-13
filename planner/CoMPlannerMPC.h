//
// Created by wenchun on 3/22/21.
//

#ifndef XIAOTIANHYBRID_COMPLANNER_H
#define XIAOTIANHYBRID_COMPLANNER_H

#include "DataSets.h"
#include "SRGB_MPC/SRGB_MPC.h"
#include "PoplarMPC/PoplarMPC.h"

// #define USE_MPC_STANCE

using Eigen::Dynamic;
using namespace Eigen;

class CoMPlannerMPC {
public:
    CoMPlannerMPC(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
                  const UserCmd *userCmd, const GaitData *gaitData, FootTaskData *footTaskData,
                  ForcesTaskData *forcesTaskData, CoMTaskData *comTaskData, const UserParameterHandler *param);

    ~CoMPlannerMPC() {
    }

    void generateHighLevelRef();

    void plan();

    void planImediate();

    void updateTaskData();

    void setExternalWrench(Vec6 wrench);

    /* for test */
    void ctModel(const Vec3 &rpy, const Vec12 &r);

    void setInitialState(const Vec3 &rpy, const Vec3 &p, const Vec3 &omega, const Vec3 &v);

    void setMPCTable(const Matrix<int, Dynamic, 1> &contactTable);

    void setReferenceTraj(const DVec &Xref);

    void solveQP();

private:

//  void dtModel();
    static inline Mat3 cross_mat(Mat3 I_inv, Vec3 r);

    void updatePostureRef();

    void generateContactTable();

    void generateRefTraj();

//  void solveQP();
    void matrix_to_real(qpOASES::real_t *dst, Matrix<double, Dynamic, Dynamic> src, s16 rows, s16 cols);

    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const UserCmd *userCmd;
    const GaitData *gaitData;
    FootTaskData *footTaskData;
    ForcesTaskData *forcesTaskData;
    CoMTaskData *comTaskData;
    const UserParameterHandler *param;

    // high level trajectory
    double xDes = 0;
    double yDes = 0;
    double maxPosError = 0.1;
    double bodyHeight = 0.35; // TODO
    double yawDes = 0.;
    double rollDes = 0.;
    double pitchDes = 0.;
    Vec3 rpInt;
    Vec3 rpCom;
    double yawdDes = 0;
    Vec12 trajInit;
    bool firstRun = true;
    size_t iter_init;

    // robot model parameters
    double mass;
    double fmax = 120.;
    Mat3 Ibody;

    Matrix<double, 13, 1> x0;
    Matrix<double, Dynamic, 1> X_d;
    Matrix<double, Dynamic, 1> U_b;
    Matrix<double, 6, 1> _ext_wrench;
    Matrix<int, Dynamic, 1> contactTable;
    Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> fmat;

    Matrix<qpOASES::real_t, Dynamic, 1> q_soln;

    SRGB_MPC::SRGB_MPC_IMPL srgbMpc;
    poplar_mpc::PoplarMPC poplarMpc;
    Vec3 vBodyDes;
    size_t _iter;

    Vec3 contactPoint[4];

    Vec3 pHipBody[4];
};

#endif //XIAOTIANHYBRID_COMPLANNER_H
