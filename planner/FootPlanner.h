//
// Created by wenchun on 3/22/21.
//

#ifndef XIAOTIANHYBRID_FOOTPLANNER_H
#define XIAOTIANHYBRID_FOOTPLANNER_H

#include "DataSets.h"
#include "FootSwingTrajectory.h"

#include "PerceptionData.h"
#include <lcm/lcm-cpp.hpp>
#include "DebugInfo_lcm.hpp"
#include "PerceptionTerrain_lcm.hpp"

class FootPlanner {
public:
    FootPlanner(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
                const UserCmd *userCmd, const GaitData *gaitData, FootTaskData *footTaskData,
                CoMTaskData *comTaskData, const PerceptionData *perceptionData, const UserParameterHandler *param);

    void plan();

private:
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const UserCmd *userCmd;
    const GaitData *gaitData;
    FootTaskData *footTaskData;
    CoMTaskData *comTaskData;
    const PerceptionData *perceptionData;
    const UserParameterHandler *param;

    FootSwingTrajectory footSwingTrajectory[4];
    Vec3 pHipBody[4];
    bool firstRun = true;
    bool firstSwing[4];
    size_t iter;

    std::array<Eigen::Matrix<double, 3, 16>, 4> M_leg;
    std::array<Eigen::Matrix<double, 3, 1>, 4> N_leg;

    std::array<Plane, 30> perception_planes;
    std::array<Plane, 4> selected_planes;
    std::array<Plane, 4> tmp_planes;
    std::list<std::array<int, 4>> plane_combinations;
    std::array<int, 4> last_plane_combination;
    std::array<int, 4> tmp_plane_combination;
    std::array<int, 4> optimal_plane_combination;
    std::array<Vec3, 4> RH_Pf;
    std::array<Vec3, 4> planned_Pf;
    std::array<Vec3, 4> planned_Pf_MRH;
    std::array<Vec3, 4> bezier_init_Pf;
    std::array<Mat3, 4> x2leg_rotations;
    lcm::LCM lcm;
    DebugInfo_lcm debug_info;

    Vec16 optimal_planned_variables;
    std::array<std::vector<int>, 4> foot_planes_id;

    double plane_shrink_meter = 0.03;
    double velx_change_limit = 1;
    double vely_change_limit = 1;
    double center_cost_consider_threshold = 1;  // only when distance(meter) between RH_Pf and center less than this threshold, the center cost will be considered
    double center_cost_weight = 1;
    double slack_of_z_in_body_frame;
    double limit_of_Pf_and_P0_in_world_z_direction = 0.2;
};


#endif //XIAOTIANHYBRID_FOOTPLANNER_H
