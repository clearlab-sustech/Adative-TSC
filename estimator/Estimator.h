//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_ESTIMATOR_H
#define XIAOTIANHYBRID_ESTIMATOR_H

#include "DataSets.h"
#include "Timer.h"

#include <lcm/lcm-cpp.hpp>
#include "StateVis.hpp"

class Estimator
{
private:
    void orientationEstimator();

    void linearKFPositionVelocityEstimator();

    void computeModelData();

    void setup();

    void pubEstimatedState();

    MeasuredState *measuredState;
    RobotModelData *robotModelData;
    EstimatedState *estimatedState;
    PerceptionData *perceptionData;
    Location *location_vision;
    const UserParameterHandler *param;

    // velocity and position estimation parameters
    Eigen::Matrix<double, 12, 1> _ps;
    Eigen::Matrix<double, 12, 1> _vs;
    Eigen::Matrix<double, 18, 18> _A;
    Eigen::Matrix<double, 18, 1> _xhat;
    Eigen::Matrix<double, 18, 18> _Q0;
    Eigen::Matrix<double, 18, 18> _P;
    Eigen::Matrix<double, 28, 28> _R0;
    Eigen::Matrix<double, 18, 3> _B;
    Eigen::Matrix<double, 28, 18> _C;

    std::mutex e_mtx;
    bool _isDone = false;

    Timer timer;

    lcm::LCM lcm_pub;
    StateVis stateVis;

protected:
    size_t init_iter;
    Quat _ori_ini_inv;

public:
    Estimator(MeasuredState *measuredState, RobotModelData *robotModelData, EstimatedState *estimatedState, PerceptionData *perceptionData, Location *location_vision, const UserParameterHandler *param);

    void estimate();

    bool isDone();

    void setContactPhase(Vec4 &phase)
    {
        estimatedState->contactPhaseDes = phase;
    }
};

#endif // XIAOTIANHYBRID_ESTIMATOR_H
