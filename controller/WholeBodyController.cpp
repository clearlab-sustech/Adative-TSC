//
// Created by wenchun on 3/22/21.
//

#include "WholeBodyController.h"
#include "orientation_tools.h"

#define BIG_NUMBER 5e10
#define MAX_TORQUE 100
#define MIN_TORQUE -100

// #define PRINT_PROBLEM_DATA

WholeBodyController::WholeBodyController(const EstimatedState *estimatedState,
                                         const RobotModelData *robotModelData,
                                         const TasksData *tasksData,
                                         const GaitData *gaitData,
                                         JointsCmd *jointsCmd,
                                         const UserParameterHandler *param) : estimatedState(estimatedState),
                                                                              robotModelData(robotModelData),
                                                                              tasksData(tasksData),
                                                                              gaitData(gaitData),
                                                                              jointsCmd(jointsCmd),
                                                                              param(param) {
    qH.resize(30, 30);
    qg.resize(30, Eigen::NoChange);
    epsilon.resize(30, Eigen::NoChange);
    A_qp.resize(46, 30);
    U_bA.resize(46, 1);
    L_bA.resize(46, 1);
    U_b.resize(30, 1);
    L_b.resize(30, 1);

    A_qp.setZero();
    qH.setZero();
    qg.setZero();
    epsilon.setZero();
    U_bA.setZero();
    L_bA.setZero();

    Jt_com.setZero();
    Jt_ori.setZero();
    Jt_feet.setZero();
    Jt_fr.setZero();

    U_b = MatrixXd::Ones(30, 1) * BIG_NUMBER;
    L_b = -MatrixXd::Ones(30, 1) * BIG_NUMBER;

    H_qpoases = (qpOASES::real_t *) malloc(30 * 30 * sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t *) malloc(30 * 1 * sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t *) malloc(30 * 46 * sizeof(qpOASES::real_t));
    lbA_qpoases = (qpOASES::real_t *) malloc(46 * 1 * sizeof(qpOASES::real_t));
    ubA_qpoases = (qpOASES::real_t *) malloc(46 * 1 * sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t *) malloc(30 * 1 * sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t *) malloc(30 * 1 * sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t *) malloc(30 * sizeof(qpOASES::real_t));
}

void WholeBodyController::runController() {
    qH.setZero();
    qg.setZero();

    Vec4Int contactState;
    for (int i(0); i < 4; i++) {
        if (gaitData->stanceTimeRemain[i] > 0.)
            contactState[i] = 1;
        else
            contactState[i] = 0;
    }

    // reorder the data
    Matrix<double, 12, 18> Jc_all;
    Matrix<double, 12, 1> Jc_dot_q_dot_all;
    for (int i = 0; i < 4; i++) {
        Jc_all.block(3 * i, 0, 3, 18) = robotModelData->Je[i]; // TODO Jc
        Jc_dot_q_dot_all.block(3 * i, 0, 3, 1) = robotModelData->Je_dot_q_dot[i]; // TODO
    }

    // Matrix<double, 22, 22> Q;
    // Matrix<double, 22, 1> R;
    // Q.setZero();
    // R.setZero();

    // Equation of Motion
    A_qp.block(0, 0, 6, 18) = robotModelData->M.block(0, 0, 6, 18);
    A_qp.block(0, 18, 6, 12) = -Jc_all.block(0, 0, 12, 6).transpose();
    L_bA.block(0, 0, 6, 1) = -robotModelData->bias.block(0, 0, 6, 1);
    U_bA.block(0, 0, 6, 1) = -robotModelData->bias.block(0, 0, 6, 1);

    // Friction
    Matrix<double, 1, 3> cx, cy, cz;
    cx << 1, 0, 0;
    cy << 0, 1, 0;
    cz << 0, 0, 1;

    for (int i = 0; i < 4; i++) {
        L_b(19 + 3 * i, 0) = 0;
        U_b(19 + 3 * i, 0) = 200 * contactState[i];

        A_qp.block(6 + 4 * i, 18 + 3 * i, 1, 3) = cx - param->mu * cz;
        A_qp.block(7 + 4 * i, 18 + 3 * i, 1, 3) = -(cx + param->mu * cz);
        A_qp.block(8 + 4 * i, 18 + 3 * i, 1, 3) = cy - param->mu * cz;
        A_qp.block(9 + 4 * i, 18 + 3 * i, 1, 3) = -(cy + param->mu * cz);
    }
    L_bA.block(6, 0, 16, 1) = -BIG_NUMBER * Vec16::Ones();
    U_bA.block(6, 0, 16, 1) = Vec16::Zero();

    // Torque Limit
    A_qp.block(22, 0, 12, 18) = robotModelData->M.block(6, 0, 12, 18);
    A_qp.block(22, 18, 12, 12) = -Jc_all.block(0, 6, 12, 12).transpose();
    L_bA.block(22, 0, 12, 1) = MIN_TORQUE * Vec12::Ones() - robotModelData->bias.block(6, 0, 12, 1);
    U_bA.block(22, 0, 12, 1) = MAX_TORQUE * Vec12::Ones() - robotModelData->bias.block(6, 0, 12, 1);

    Matrix<double, 12, 12> pmatrix;
    pmatrix.setIdentity();
    for (int i(0); i < 4; i++)
        pmatrix.block(3 * i, 0, 3, 12) = contactState[i] * pmatrix.block(3 * i, 0, 3, 12);

    A_qp.block(34, 0, 12, 18) = pmatrix * Jc_all;
    L_bA.block(34, 0, 12, 1) = -pmatrix * Jc_dot_q_dot_all;
    U_bA.block(34, 0, 12, 1) = -pmatrix * Jc_dot_q_dot_all;

    // CoM position task
    //TODO need J_fb and J_fb_dot
    Jt_com.block(0, 0, 3, 3) = estimatedState->floatingBaseState.R_wb;
    Vec3 xdd_com_ref = tasksData->comTaskData.linAccWorld;
//            Kp_com * Mat3::Identity() * (tasksData->comTaskData.pos - estimatedState->floatingBaseState.pos) +
//            Kd_com * Mat3::Identity() * (tasksData->comTaskData.vWorld - estimatedState->floatingBaseState.vWorld);
    qH += Jt_com.transpose() * W_com * Jt_com;
    qg += -Jt_com.transpose() * W_com * xdd_com_ref;

    // CoM orientation task
    Jt_ori.block(0, 3, 3, 3) = estimatedState->floatingBaseState.R_wb;
    Mat3 R_des = ori::rpyToRotMat(tasksData->comTaskData.rpy); // TODO
    Vec3 xdd_ori_ref = tasksData->comTaskData.angAccWorld;
//            Kp_ori * Mat3::Identity() * pin::log3(R_des * estimatedState->floatingBaseState.R_wb.transpose()) +
//            Kd_ori * Mat3::Identity() *
//            (tasksData->comTaskData.omegaWorld - estimatedState->floatingBaseState.omegaWorld);
    qH += Jt_ori.transpose() * W_ori * Jt_ori;
    qg += -Jt_ori.transpose() * W_ori * xdd_ori_ref;

    // Wheel position task
    //TODO check the dimension of the task data
    for (int i = 0; i < 4; i++) {
        Jt_feet.block(0, 0, 3, 18) = robotModelData->Je[i];
        qH += Jt_feet.transpose() * W_w * Jt_feet;
        qg += -Jt_feet.transpose() * W_w * ((tasksData->footTaskData[i].linAccWorld - robotModelData->Je_dot_q_dot[i] +
                                             Kp_w * Mat3::Identity() *
                                             (tasksData->footTaskData[i].pos - estimatedState->footState[i].pos) +
                                             Kd_w * Mat3::Identity() * (tasksData->footTaskData[i].vWorld -
                                                                        estimatedState->footState[i].vWorld)));
    }

#ifdef PRINT_PROBLEM_DATA
    std::cout << "A_lb:\n" << L_bA << "\n";
    std::cout << "A_ub:\n" << U_bA << "\n";
    // std::cout << "A:\n" << A_qp << "\n";

    // task data
    std::cout << "com pos ref: " << tasksData->comTaskData.pos.transpose() << "\n";
    std::cout << "com vel ref: " << tasksData->comTaskData.vWorld.transpose() << "\n";
    for (int i(0); i<4; i++) {
        std::cout << "foot pos ref " << i << ": " << tasksData->footTaskData[i].pos.transpose() << "\n";
        std::cout << "foot vel ref " << i << ": " << tasksData->footTaskData[i].vWorld.transpose() << "\n";
    }
#endif

    int num_cons = 46;
    int num_var = 30;

    // eigen to qpOASES
    // std::copy(qH.data(), qH.data() + 34 * 34, H_qpoases);
    // std::copy(qg.data(), qg.data() + 34 * 1, g_qpoases);
    // std::copy(A_qp.data(), A_qp.data() + 34 * 46, A_qpoases);
    // std::copy(U_bA.data(), U_bA.data() + 46, ubA_qpoases);
    // std::copy(L_bA.data(), L_bA.data() + 46, lbA_qpoases);
    // std::copy(L_b.data(), L_b.data() + 34, lb_qpoases);
    // std::copy(U_b.data(), U_b.data() + 34, ub_qpoases);
    matrix_to_real(H_qpoases, qH, 30, 30);
    matrix_to_real(g_qpoases, qg, 30, 1);
    matrix_to_real(A_qpoases, A_qp, 46, 30);
    matrix_to_real(ubA_qpoases, U_bA, 46, 1);
    matrix_to_real(lbA_qpoases, L_bA, 46, 1);
    matrix_to_real(ub_qpoases, U_b, 30, 1);
    matrix_to_real(lb_qpoases, L_b, 30, 1);

    qpOASES::int_t nWSR = 1000;

    // solve the problem
    qpOASES::QProblem prob_ori(num_var, num_cons);
    qpOASES::Options opt;
    opt.setToMPC();
    opt.printLevel = qpOASES::PL_NONE;
    prob_ori.setOptions(opt);
    // prob_ori.init(H_qpoases, g_qpoases, A_qpoases, lb_qpoases, ub_qpoases, lbA_qpoases, ubA_qpoases, nWSR);
    prob_ori.init(H_qpoases, g_qpoases, A_qpoases, lb_qpoases, ub_qpoases, lbA_qpoases, ubA_qpoases, nWSR);
    int rval = prob_ori.getPrimalSolution(q_soln);
    if (rval == qpOASES::SUCCESSFUL_RETURN)
        for (int i(0); i < 30; i++)
            epsilon[i] = q_soln[i];
    else
        printf("wbc qp failed to solve!\n");

//    std::cout << "wbc forces: \n" << epsilon.tail(12).transpose() << "\n";

    MatrixXd S(12, 30);
    S << robotModelData->M.block(6, 0, 12, 18), -Jc_all.block(0, 6, 12, 12).transpose();
    jointsCmd->tau_ff = S * epsilon + robotModelData->bias.tail(12);


    /*jointsCmd->tau_ff = -Jc_all.block(0, 6, 12, 12).transpose() * tasksData->forcesTaskData.forces_ref +
                        1.0 * robotModelData->generalizedGravity.tail(12);*/
//    std::cout << "Joint torque: \n" << jointsCmd->tau_ff.transpose() << "\n";
    // getchar();
}

void
WholeBodyController::matrix_to_real(qpOASES::real_t *dst, Matrix<double, Dynamic, Dynamic> src, s16 rows, s16 cols) {
    s32 a = 0;
    for (s16 r = 0; r < rows; r++) {
        for (s16 c = 0; c < cols; c++) {
            dst[a] = src(r, c);
            a++;
        }
    }
}
