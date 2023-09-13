//
// Created by wenchun on 3/22/21.
//

#ifndef XIAOTIANHYBRID_WHOLEBODYCONTROLLER_H
#define XIAOTIANHYBRID_WHOLEBODYCONTROLLER_H

#include "DataSets.h"
#include "qpOASES.hpp"

using namespace Eigen;

class WholeBodyController {
public:
  WholeBodyController(const EstimatedState *estimatedState,
                      const RobotModelData *robotModelData,
                      const TasksData *tasksData,
                      const GaitData *gaitData,
                      JointsCmd *jointsCmd,
                      const UserParameterHandler *param);

  void runController();

private:
  void matrix_to_real(qpOASES::real_t *dst, Matrix<double, Dynamic, Dynamic> src, s16 rows, s16 cols);

  const EstimatedState *estimatedState;
  const RobotModelData *robotModelData;
  const TasksData *tasksData;
  const GaitData *gaitData;
  JointsCmd *jointsCmd;
  const UserParameterHandler *param;

  // controller parameters
  double Kp_cp = 200;
  double Kd_cp = 10;
  Mat3 W_cp = 10 * Mat3::Identity();
  double Kp_com = 800;
  double Kd_com = 80;
  Mat3 W_com = 50 * Mat3::Identity();
  double Kp_ori = 800;
  double Kd_ori = 80;
  Mat3 W_ori = 50 * Mat3::Identity();
  double Kp_w = 1000;
  double Kd_w = 80;
  Mat3 W_w = 40 * Mat3::Identity();
  Matrix<double,3,30> Jt_com, Jt_ori, Jt_feet, Jt_fr;

  Matrix<double,Dynamic,Dynamic> qH;
  Matrix<double,Dynamic,1> qg;
  Matrix<double,Dynamic,1> epsilon;
  Matrix<double,Dynamic,30> A_qp;
  Matrix<double,Dynamic,1> U_bA;
  Matrix<double,Dynamic,1> L_bA;
  Matrix<double,Dynamic,1> U_b;
  Matrix<double,Dynamic,1> L_b;

  qpOASES::real_t* H_qpoases;
  qpOASES::real_t* g_qpoases;
  qpOASES::real_t* A_qpoases;
  qpOASES::real_t* lb_qpoases;
  qpOASES::real_t* ub_qpoases;
  qpOASES::real_t* lbA_qpoases;
  qpOASES::real_t* ubA_qpoases;
  qpOASES::real_t* q_soln;
};


#endif //XIAOTIANHYBRID_WHOLEBODYCONTROLLER_H
