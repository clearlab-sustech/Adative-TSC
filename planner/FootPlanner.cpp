//
// Created by wenchun on 3/22/21.
//

#include "FootPlanner.h"
#include "orientation_tools.h"
#include "qpOASES.hpp"
#include "Timer.h"

// #define FOOTHOLD_PLANNER_ON // if using new foothold planner

using namespace ori;

class TerrainHandler
{
public:
	~TerrainHandler() {}
};

FootPlanner::FootPlanner(const EstimatedState *estimatedState,
						 const RobotModelData *robotModelData,
						 const UserCmd *userCmd,
						 const GaitData *gaitData,
						 FootTaskData *footTaskData,
						 CoMTaskData *comTaskData,
						 const PerceptionData *perceptionData,
						 const UserParameterHandler *param) : estimatedState(estimatedState),
															  robotModelData(robotModelData),
															  userCmd(userCmd),
															  gaitData(gaitData),
															  footTaskData(footTaskData),
															  comTaskData(comTaskData),
															  perceptionData(perceptionData),
															  param(param),
															  iter(0)
{
	for (int i(0); i < 4; i++)
	{
		firstSwing[i] = true;
		pHipBody[i] = Vec3((i < 2) ? 0.234701 : -0.2336, (i == 0 || i == 2) ? 0.13 : -0.13, 0);
		x2leg_rotations[i].setIdentity();
	}

	if (!lcm.good())
		printf("lcm not good!");
}
void FootPlanner::plan()
{
	perception_planes = perceptionData->planes;
	if (firstRun)
	{
		for (int i = 0; i < 4; i++)
		{
			footSwingTrajectory[i].setHeight(param->swing_height);
			footSwingTrajectory[i].setInitialPosition(estimatedState->footState[i].pos);
			footSwingTrajectory[i].setFinalPosition(estimatedState->footState[i].pos);
		}
		firstRun = false;
	}

	Vec3 vBodyDes(userCmd->vx_des, userCmd->vy_des, 0.);
	Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * vBodyDes;

#ifndef FOOTHOLD_PLANNER_ON
	if (iter % 1 == 0)
	{
		// compute foot placement
		for (int i(0); i < 4; i++)
		{
			footSwingTrajectory[i].setHeight(param->swing_height);
			double stanceTime = gaitData->nextStanceTime[i];
			Vec3 pYawCorrected =
				coordinateRotation(CoordinateAxis::Z, -userCmd->yawd_des * stanceTime / 2) * pHipBody[i];

			/* 	Vec3 Pf = estimatedState->floatingBaseState.pos + estimatedState->floatingBaseState.R_wb *
			(pYawCorrected + Vec3(userCmd->vx_des, userCmd->vy_des, 0) * gaitData->swingTimeRemain[i]); */
			planned_Pf_MRH[i] = estimatedState->floatingBaseState.pos + estimatedState->floatingBaseState.R_wb *
																			(pYawCorrected + estimatedState->floatingBaseState.vBody * gaitData->swingTimeRemain[i]);

			double p_rel_max = 0.3f;
			double p_rel_y_max = 0.2f;

			// Using the estimated velocity is correct
			// double pfx_rel = estimatedState->floatingBaseState.vWorld[0] * 0.5 * gaitData->nextStanceTime[i] +
			// 			 .03f * (estimatedState->floatingBaseState.vWorld[0] - vWorldDes[0]) +
			// 			 (0.5f * estimatedState->floatingBaseState.pos[2] / 9.81f) *
			// 			 (estimatedState->floatingBaseState.vWorld[1] * userCmd->yawd_des);
			double pfx_rel = vWorldDes[0] * 0.5 * gaitData->nextStanceTime[i] +
							 .03f * (estimatedState->floatingBaseState.vWorld[0] - vWorldDes[0]) +
							 (0.5f * estimatedState->floatingBaseState.pos[2] / 9.81f) *
								 (estimatedState->floatingBaseState.vWorld[1] * userCmd->yawd_des);

			//        std::cout << "pfx_rel: " << pfx_rel << ", " << userCmd->vx_des * 0.5 * gaitData->nextStanceTime[i] << std::endl;

			double pfy_rel = estimatedState->floatingBaseState.vWorld[1] * .5 * gaitData->nextStanceTime[i] +
							 .03f * (estimatedState->floatingBaseState.vWorld[1] - vWorldDes[1]) +
							 (0.5f * estimatedState->floatingBaseState.pos[2] / 9.81f) *
								 (-estimatedState->floatingBaseState.vWorld[0] * userCmd->yawd_des);
			pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
			pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_y_max), p_rel_y_max);
			planned_Pf_MRH[i][0] += pfx_rel;
			planned_Pf_MRH[i][1] += pfy_rel;
			// planned_Pf_MRH[i][2] -= (param->body_height - 0.02);
			planned_Pf_MRH[i][2] = 0.02;

			if (iter > 0)
			{
				planned_Pf[i] = footTaskData[i].nextContactPos;
			}
			else
			{
				planned_Pf[i] = planned_Pf_MRH[i];
			}

			footSwingTrajectory[i].usingDefaultHeight();
		}
		comTaskData->footPlannedVBody = vBodyDes;
	}

#endif

	Vec4 V_des;
	std::array<std::vector<int>, 4> foot_planes_id_big;

	double min_objv = 1000;

	Eigen::Matrix<double, 16, 16, Eigen::RowMajor> H;
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> H_limit_a;
	Eigen::Matrix<double, 16, 1> g;
	Mat23 Select_1;
	Vec3 Select_2(0, 0, 1);
	Vec16 planed_variables;
	bool is_qp_solved = false;

	std::array<std::vector<Vec3>, 4> controlPoints;
	int swing_planner_success_num = 4;

	double planer_time_cost = 0;

#ifdef FOOTHOLD_PLANNER_ON

	if (iter % 5 == 0)
	{
		Timer tc;
		V_des << vBodyDes[0], vBodyDes[1], vBodyDes[0], vBodyDes[1];

		// update Pf(v) model
		for (int i = 0; i < 4; i++)
		{
			auto &estBase = estimatedState->floatingBaseState;
			M_leg[i].setZero();
			if (gaitData->swingTimeRemain[i] > 0)
			{
				M_leg[i].block<3, 2>(0, 0) = estBase.R_wb.topLeftCorner(3, 2) * gaitData->nextStanceTime[i] * 0.5;
				M_leg[i].block<3, 3>(0, 4 + 3 * i) = estBase.R_wb.topLeftCorner(3, 3);
				N_leg[i] = estBase.pos + estimatedState->floatingBaseState.R_wb * pHipBody[i] + Vec3(0, 0, -param->body_height) + estBase.vWorld * gaitData->swingTimeRemain[i];
			}
			if (gaitData->stanceTimeRemain[i] > 0)
			{
				M_leg[i].block<3, 2>(0, 0) = estBase.R_wb.topLeftCorner(3, 2) * gaitData->swingTime[i];
				M_leg[i].block<3, 2>(0, 2) = estBase.R_wb.topLeftCorner(3, 2) * gaitData->nextStanceTime[i] * 0.5;
				M_leg[i].block<3, 3>(0, 4 + 3 * i) = estBase.R_wb.topLeftCorner(3, 3);
				N_leg[i] = estBase.pos + estimatedState->floatingBaseState.R_wb * pHipBody[i] + Vec3(0, 0, -param->body_height) + estBase.vWorld * gaitData->stanceTimeRemain[i];
			}
		}

		// plane pre-selection
		for (int n = 0; n < 4; n++)
		{
			foot_planes_id[n].clear();
			if (gaitData->stanceTimeRemain[n] == 0 && gaitData->swingTimeRemain[n] < 0.8 * gaitData->swingTime[n])
			{
				foot_planes_id[n].push_back(-1);
			}
			else
			{
				RH_Pf[n] = M_leg[n].topLeftCorner(3, 4) * V_des + N_leg[n];
				Vec3 Pf = RH_Pf[n];
				int count = -1;
				for (auto plane : perception_planes)
				{
					count++;
					if (plane.bound_num == 0)
						continue;
					double point_plane_distance_n = abs(plane.A[0] * Pf[0] + plane.A[1] * Pf[1] - Pf[2] + plane.A[2]) / sqrt(plane.A[0] * plane.A[0] + plane.A[1] * plane.A[1] + 1);
					if (point_plane_distance_n < 0.4)
					{
						double point_plane_distance_l = 0;
						bool inside_plane = true;
						bool inside_line = false;
						for (int i = 0; i < plane.bound_num; i++)
						{
							if (plane.Bs[i].head(2).transpose() * Pf.head(2) > plane.Bs[i][2])
								inside_plane = false;
						}
						if (!inside_plane)
						{
							point_plane_distance_l = 1000;
							for (int i = 0; i < plane.bound_num; i++)
							{
								Vec2 vec_tmp_1 = plane.Vertexs[(i + 1) % plane.bound_num] - plane.Vertexs[i];
								Vec2 vec_tmp_2 = Pf.head(2) - plane.Vertexs[i];
								inside_line = false;
								if (vec_tmp_2.dot(vec_tmp_1) > 0)
								{
									vec_tmp_2 = Pf.head(2) - plane.Vertexs[(i + 1) % plane.bound_num];
									if (vec_tmp_2.dot(-vec_tmp_1) > 0)
									{
										inside_line = true;
									}
								}
								if (inside_line)
								{
									double distance_tmp = abs(plane.Bs[(i + 1) % plane.bound_num].head(2).transpose() * Pf.head(2) - plane.Bs[(i + 1) % plane.bound_num][2]) / plane.Bs[(i + 1) % plane.bound_num].head(2).norm();
									point_plane_distance_l = std::min(point_plane_distance_l, distance_tmp);
								}
								else
								{
									double distance_tmp = std::min((Pf.head(2) - plane.Vertexs[i]).norm(),
																   (Pf.head(2) - plane.Vertexs[(i + 1) % plane.bound_num]).norm());
									point_plane_distance_l = std::min(point_plane_distance_l, distance_tmp);
								}
							}
						}
						if (point_plane_distance_l < 0.05)
						{
							foot_planes_id[n].push_back(count);
						}
						if (point_plane_distance_l < 0.4)
						{
							foot_planes_id_big[n].push_back(count);
						}
					}
				}
				// if no plane pre-selected, select the last one;
				if (foot_planes_id[n].size() == 0)
				{
					foot_planes_id[n].push_back(optimal_plane_combination[n]);
				}
			}
		}

		// foothold planning
		// assign plane combinations
		plane_combinations.clear();
		for (auto f1 : foot_planes_id[0])
		{
			for (auto f2 : foot_planes_id[1])
			{
				for (auto f3 : foot_planes_id[2])
				{
					for (auto f4 : foot_planes_id[3])
					{
						std::array<int, 4> located_plane_id = {f1, f2, f3, f4};
						plane_combinations.push_back(located_plane_id);
					}
				}
			}
		}

		// solving qp planner

		Select_1 << 1, 0, 0,
			0, 1, 0;
		slack_of_z_in_body_frame = std::max(param->body_height - 0.1, 0.4);

		auto comb_iter = plane_combinations.begin();
		for (auto i_comb = 0; i_comb < plane_combinations.size(); i_comb++)
		{
			H.setZero();
			g.setZero();
			// set velocity close to Vdes cost
			H.topLeftCorner(4, 4).setIdentity();
			g.head(4) = -H.topLeftCorner(4, 4) * V_des;
			// set next 2 step velocity closer to first step velocity cost
			H_limit_a << 1, 0, -1, 0,
				0, 1, 0, -1,
				-1, 0, 1, 0,
				0, -1, 0, 1;
			H.topLeftCorner(4, 4) += H_limit_a;
			// set slack variables smaller cost
			// H.bottomRightCorner(12, 12).diagonal() << 0.01, 0.01, 0.01,
			// 										  0.01, 0.01, 0.01,
			// 										  0.01, 0.01, 0.01,
			// 										  0.01, 0.01, 0.01;
			H.bottomRightCorner(12, 12).setIdentity();

			auto located_plane_id = *comb_iter;
			comb_iter++;

			for (int i_leg = 0; i_leg < 4; i_leg++)
			{
				if (located_plane_id[i_leg] == -1)
				{
					tmp_planes[i_leg] = selected_planes[i_leg];
				}
				else
				{
					tmp_planes[i_leg] = perception_planes[located_plane_id[i_leg]];
				}
			}

			// added step closer the center of the plane cost
			Vec3 Select_3_vec(1, 0, 0);
			Mat3 Select_3;
			Select_3 = Select_3_vec * Select_3_vec.transpose();
			std::list<int> to_added_center_cost = {0, 1, 2, 3};

			if (located_plane_id[0] == located_plane_id[3])
			{
				to_added_center_cost.remove(0);
				to_added_center_cost.remove(3);
			}
			if (located_plane_id[2] == located_plane_id[1])
			{
				to_added_center_cost.remove(2);
				to_added_center_cost.remove(1);
			}

			for (auto i_center : to_added_center_cost)
			{
				Vec3 Pmid(0, 0, 0);
				int _bound_num = tmp_planes[i_center].bound_num;
				for (int i_bound = 0; i_bound < _bound_num; i_bound++)
				{
					Pmid.head(2) += tmp_planes[i_center].Vertexs[i_bound];
				}
				Pmid = Pmid / _bound_num;
				if (Select_3_vec.transpose() * (RH_Pf[i_center] - Pmid) > center_cost_consider_threshold)
					continue;
				// if (located_plane_id[i_center]==12)
				// 	continue;
				H += 1 * center_cost_weight * M_leg[i_center].transpose() * Select_3 * M_leg[i_center];
				g += 1 * center_cost_weight * M_leg[i_center].transpose() * Select_3 * (N_leg[i_center] - Pmid);
			}

			int total_bound_num = 0;
			for (int i_leg = 0; i_leg < 4; i_leg++)
			{
				total_bound_num += tmp_planes[i_leg].bound_num;
			}
			// total_bound_num : plane xy inequality constrains
			// 4 : plane z equality constrains
			// 2 : next 2 steps velocity change constrains
			// 4 : Pf can't be too far from P0(init foothold) in z direction
			int constrain_num = total_bound_num + 4 + 2 + 4;
			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> A(constrain_num, 16);
			Eigen::VectorXd ubA(constrain_num);
			Eigen::VectorXd lbA(constrain_num);
			Eigen::VectorXd ub(16);
			Eigen::VectorXd lb(16);
			A.setZero();
			for (int i = 0; i < 2; i++)
			{
				ub[2 * i] = estimatedState->floatingBaseState.vBody[0] + velx_change_limit * (i + 1);
				ub[2 * i + 1] = estimatedState->floatingBaseState.vBody[1] + vely_change_limit * (i + 1);
				lb[2 * i] = estimatedState->floatingBaseState.vBody[0] - velx_change_limit * (i + 1);
				lb[2 * i + 1] = estimatedState->floatingBaseState.vBody[1] - velx_change_limit * (i + 1);
			}
			for (int i = 4; i < 16; i++)
			{
				if ((i - 4) % 3 == 2)
				{
					ub[i] = slack_of_z_in_body_frame;
					lb[i] = -slack_of_z_in_body_frame;
				}
				else
				{
					ub[i] = 0.06;
					lb[i] = -0.06;
				}
			}

			int count = 0;
			for (int i = 0; i < 4; i++)
			{
				auto &bound_num = tmp_planes[i].bound_num;
				auto &Bs = tmp_planes[i].Bs;
				for (int k = 0; k < bound_num; k++)
				{
					A.row(count) = Bs[k].head(2).transpose() * Select_1 * M_leg[i];
					ubA[count] = Bs[k][2] - Bs[k].head(2).transpose() * Select_1 * N_leg[i] - plane_shrink_meter * Bs[k].head(2).norm();
					lbA[count] = -1000;
					count++;
				}
				auto &plane_A = tmp_planes[i].A;
				A.row(total_bound_num + i) = (Select_2.transpose() - plane_A.head(2).transpose() * Select_1) * M_leg[i];
				ubA[total_bound_num + i] = plane_A[2] - (Select_2.transpose() - plane_A.head(2).transpose() * Select_1) * N_leg[i];
				lbA[total_bound_num + i] = ubA[total_bound_num + i];
			}
			A.row(total_bound_num + 4 + 0).head(4) << -1, 0, 1, 0;
			A.row(total_bound_num + 4 + 1).head(4) << 0, -1, 0, 1;
			ubA[total_bound_num + 4 + 0] = velx_change_limit;
			ubA[total_bound_num + 4 + 1] = vely_change_limit;
			lbA[total_bound_num + 4 + 0] = -velx_change_limit;
			lbA[total_bound_num + 4 + 1] = -vely_change_limit;
			for (int i = 0; i < 4; i++)
			{
				A.row(total_bound_num + 4 + 2 + i) = M_leg[i].row(2);
				ubA[total_bound_num + 4 + 2 + i] = bezier_init_Pf[i][2] - N_leg[i][2] + limit_of_Pf_and_P0_in_world_z_direction;
				lbA[total_bound_num + 4 + 2 + i] = bezier_init_Pf[i][2] - N_leg[i][2] - limit_of_Pf_and_P0_in_world_z_direction;
			}
			qpOASES::QProblem solver(planed_variables.size(), constrain_num);
			qpOASES::int_t nWSR = 20;
			qpOASES::Options opt;
			opt.setToMPC();
			opt.printLevel = qpOASES::PL_NONE;
			solver.setOptions(opt);
			solver.init(H.data(), g.data(), A.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);
			if (solver.isSolved())
			{
				solver.getPrimalSolution(planed_variables.data());
				double tmp_objv = solver.getObjVal();
				if (tmp_objv < min_objv)
				{
					min_objv = tmp_objv;
					optimal_planned_variables = planed_variables;
					optimal_plane_combination = located_plane_id;
					is_qp_solved = true;
				}
			}
		}

		if (is_qp_solved)
		{
			// update velocity and foothood by new qp result
			comTaskData->footPlannedVBody.head(2) = optimal_planned_variables.head(2);
			comTaskData->footPlannedVBody[2] = 0;
			for (int i = 0; i < 4; i++)
			{
				planned_Pf[i] = M_leg[i] * optimal_planned_variables + N_leg[i] + Vec3(0, 0, -0.01);
				if (optimal_plane_combination[i] != -1)
				{
					selected_planes[i] = perception_planes[optimal_plane_combination[i]];
				}
			}
		}
		else
		{
			comTaskData->footPlannedVBody << 0, 0, 0;
			// do not update anything
			// TODO: figuring out why could fail
			// printf("foothold planner failed!!!\n");
		}

		// to find last standed plane
		for (int i = 0; i < 4; i++)
		{
			if (gaitData->swingTimeRemain[i] > 0 && gaitData->swingTimeRemain[i] < 0.1)
			{
				tmp_plane_combination[i] = optimal_plane_combination[i];
			}
			if (gaitData->stanceTimeRemain[i] > 0)
			{
				last_plane_combination[i] = tmp_plane_combination[i];
			}
		}

		// compute rotation of each leg path
		for (int i = 0; i < 4; i++)
		{
			// if (last_plane_combination[i] != optimal_plane_combination[i])
			if (false)
			{
				Vec3 path_vector = planned_Pf[i] - bezier_init_Pf[i];
				Vec3 projection_vector(path_vector[0], path_vector[1], 0);
				Mat3 yaw_rotation(Eigen::Quaterniond().setFromTwoVectors(Vec3(1, 0, 0), projection_vector));
				Mat3 pitch_rotation(Eigen::Quaterniond().setFromTwoVectors(projection_vector, path_vector));
				Mat3 x2leg_rotation(yaw_rotation * pitch_rotation);
				x2leg_rotations[i] = x2leg_rotation;
			}
			else
			{
				x2leg_rotations[i].setIdentity();
			}
		}

		// To find control points (how height the terrain on the path)
		for (int i = 0; i < 4; i++)
		{
			// if (last_plane_combination[i] != optimal_plane_combination[i])
			if (false)
			{
				Vec3 foot_now = bezier_init_Pf[i];
				Vec3 Pf = planned_Pf[i];
				Mat2 CrossoverA;
				Vec2 Crossoverb;
				CrossoverA.row(0) << Pf.y() - foot_now.y(), foot_now.x() - Pf.x();
				Crossoverb[0] = (Pf.y() - foot_now.y()) * foot_now.x() + (foot_now.x() - Pf.x()) * foot_now.y();
				for (int k = 0; k < foot_planes_id_big[i].size() + 1; k++)
				{
					int index;
					if (k == 0)
					{
						index = last_plane_combination[i];
					}
					else
					{
						if (foot_planes_id_big[i][k - 1] != last_plane_combination[i])
						{
							index = foot_planes_id_big[i][k - 1];
						}
						else
						{
							continue;
						}
					}
					auto plane = perception_planes[index];
					for (int n = 0; n < plane.bound_num; n++)
					{
						CrossoverA.row(1) = plane.Bs[(n + 1) % plane.bound_num].head(2);
						Crossoverb[1] = plane.Bs[(n + 1) % plane.bound_num][2];
						// std::cout << "\nCrossoverA :\n" << CrossoverA << "\nCrossoverb :\n" << Crossoverb << std::endl;
						if (CrossoverA.determinant() != 0)
						{
							Vec2 crossPoint = CrossoverA.inverse() * Crossoverb; // 5e-5 ms
							// std::cout << "cross point: " << crossPoint << std::endl;
							Vec2 tmpVec1 = (crossPoint - plane.Vertexs[n]).cwiseProduct(crossPoint - plane.Vertexs[(n + 1) % plane.bound_num]);
							auto tmp1 = tmpVec1[0] < 0 || tmpVec1[1] < 0;
							Vec2 tmpVec2 = (crossPoint - Pf.head(2)).cwiseProduct(crossPoint - foot_now.head(2));
							auto tmp2 = tmpVec2[0] < 0 || tmpVec2[1] < 0;
							if (tmp1 && tmp2)
							{
								Vec3 oneControlPoint;
								oneControlPoint.head(2) = crossPoint;
								oneControlPoint[2] = plane.A.head(2).transpose() * crossPoint + plane.A[2];
								controlPoints[i].push_back(x2leg_rotations[i].transpose() * oneControlPoint);
							}
						}
					}
				}
			}
		}
		// plan swing path
		for (int i = 0; i < 4; i++)
		{
			// if (last_plane_combination[i] != optimal_plane_combination[i])
			if (false)
			{
				Vec3 path_vector = planned_Pf[i] - bezier_init_Pf[i];
				// if planned foothold is too closed to init foothold, then don't compute rotation by two points (don't rotate)
				Vec3 foot_init = x2leg_rotations[i].transpose() * bezier_init_Pf[i];
				Vec3 Pf = x2leg_rotations[i].transpose() * planned_Pf[i];

				// std::cout << "foot_now: " << foot_now << "Pf: " << Pf << std::endl;
				double xe0 = foot_init[0];
				double xe1 = Pf[0];
				double ze0 = foot_init[2];
				double ze1 = Pf[2];
				int control_num = controlPoints[i].size();
				if (control_num == 0)
				{
					Vec3 tmp_mid = (foot_init + Pf) / 2;
					controlPoints[i].push_back(tmp_mid);
					control_num = 1;
				}
				for (int n = 0; n < control_num; n++)
				{
					double tmp_ratio = (controlPoints[i][n][0] - foot_init[0]) / (Pf[0] - foot_init[0]);
					// std::cout << "tmp ratio: " << tmp_ratio << std::endl;
					controlPoints[i][n][2] += 0.05 * 4 * (0.25 - (tmp_ratio - 0.5) * (tmp_ratio - 0.5));
				}
				DVec swing_coefs(4);
				swing_coefs.setZero();

				Eigen::Matrix<double, -1, -1, Eigen::RowMajor> Hess_swing(4, 4);
				DVec g0_swing(4);
				Eigen::Matrix<double, -1, -1, Eigen::RowMajor> CE_swing(2, 4);
				DVec ce0_swing(2);
				Eigen::Matrix<double, -1, -1, Eigen::RowMajor> CI_swing(2 + control_num, 4);
				DVec ci0_swing(2 + control_num);

				Hess_swing = Vec4(3 * xe0 * xe0, 2 * xe0, 1, 0.1) * Vec4(3 * xe0 * xe0, 2 * xe0, 1, 0.1).transpose() + Vec4(3 * xe1 * xe1, 2 * xe1, 1, 0.1) * Vec4(3 * xe1 * xe1, 2 * xe1, 1, 0.1).transpose();
				g0_swing.setZero();
				CE_swing.row(0) = Vec4(pow(xe0, 3), pow(xe0, 2), xe0, 1).transpose();
				CE_swing.row(1) = Vec4(pow(xe1, 3), pow(xe1, 2), xe1, 1).transpose();
				// CE_swing.row(2) = Vec4(3*xe1*xe1, 2*xe1, 1, 0).transpose();
				ce0_swing << -ze0, -ze1;
				CI_swing.row(0) = Vec4(3 * xe0 * xe0, 2 * xe0, 1, 0).transpose();
				CI_swing.row(1) = Vec4(-3 * xe1 * xe1, -2 * xe1, -1, 0).transpose();
				ci0_swing.head(2) << 0, 0;
				for (int n = 0; n < control_num; n++)
				{
					CI_swing.row(2 + n) = Vec4(pow(controlPoints[i][n][0], 3), pow(controlPoints[i][n][0], 2), controlPoints[i][n][0], 1).transpose();
					ci0_swing[2 + n] = -(controlPoints[i][n][2]);
				}

				Eigen::Matrix<double, -1, -1, Eigen::RowMajor> A_swing(2 + 2 + control_num, 4);
				DVec lbA_swing(2 + 2 + control_num);
				DVec ubA_swing(2 + 2 + control_num);
				A_swing.topLeftCorner(2, 4) = CE_swing;
				lbA_swing.head(2) = -ce0_swing;
				ubA_swing.head(2) = -ce0_swing;
				A_swing.bottomLeftCorner(2 + control_num, 4) = CI_swing;
				lbA_swing.tail(2 + control_num) = -ci0_swing;
				ubA_swing.tail(2 + control_num).fill(10000);

				qpOASES::QProblem solver(swing_coefs.size(), 2 + 2 + control_num);
				qpOASES::int_t nWSR = 20;
				qpOASES::Options opt;
				opt.setToMPC();
				opt.printLevel = qpOASES::PL_NONE;
				solver.setOptions(opt);
				// std::cout << "\nH: \n" << Hess_swing  << "\ng: \n" << g0_swing  << "\nA: \n" << A_swing
				//         << "\nlbA: \n" << lbA_swing  << "\nubA: \n" << ubA_swing << std::endl;
				solver.init(Hess_swing.data(), g0_swing.data(), A_swing.data(), nullptr, nullptr, lbA_swing.data(), ubA_swing.data(), nWSR);
				if (solver.isSolved())
				{
					solver.getPrimalSolution(swing_coefs.data());
					// footSwingTrajectory[i].notUsingDefaultHeight();
					footSwingTrajectory[i].setPathCoefs(swing_coefs);
				}
				else
				{
					swing_planner_success_num--;
					footSwingTrajectory[i].usingDefaultHeight();
					// TODO: figuring out when it would fail
				}
			}
			else
			{
				swing_planner_success_num--;
				footSwingTrajectory[i].usingDefaultHeight();
			}
		}

		debug_info.foothold_planner_state = is_qp_solved;
		debug_info.swing_planner_state = swing_planner_success_num;

		planer_time_cost = tc.getMs();
	}
#endif

	// estimate planned pitch angle
	double ffx = gaitData->stanceTimeRemain[1] > 0 ? planned_Pf[1][0] : planned_Pf[0][0];
	double fhx = gaitData->stanceTimeRemain[3] > 0 ? planned_Pf[3][0] : planned_Pf[2][0];
	double ffz = gaitData->stanceTimeRemain[1] > 0 ? planned_Pf[1][2] : planned_Pf[0][2];
	double fhz = gaitData->stanceTimeRemain[3] > 0 ? planned_Pf[3][2] : planned_Pf[2][2];
	comTaskData->pitchDes = -atan((ffz - fhz) / (ffx - fhx));

	// generating foot trajectory
	for (int i = 0; i < 4; i++)
	{
		footSwingTrajectory[i].setFinalPosition(x2leg_rotations[i].transpose() * planned_Pf[i]);
		if (gaitData->swingTimeRemain[i] > 0.) // swing
		{
			if (firstSwing[i])
			{
				firstSwing[i] = false;
				footSwingTrajectory[i].setInitialPosition(x2leg_rotations[i].transpose() * estimatedState->footState[i].pos);
				bezier_init_Pf[i] = estimatedState->footState[i].pos;
			}
			footSwingTrajectory[i].setInitialPosition(x2leg_rotations[i].transpose() * bezier_init_Pf[i]);
			double swingPhase = 1 - gaitData->swingTimeRemain[i] / gaitData->swingTime[i];

			if (gaitData->late_contact[i])
			{
				footTaskData[i].pos = estimatedState->footState[i].pos - Vec3(0, 0, 0.05);
				footTaskData[i].vWorld = Vec3(0, 0, -2.0);
				footTaskData[i].linAccWorld.setZero();
				// footTaskData[i].nextContactPos = estimatedState->footState[i].pos;
			}
			else if (estimatedState->footState[i].contact_detected && swingPhase > 0.5)
			{
				footTaskData[i].pos = estimatedState->footState[i].pos;
				footTaskData[i].vWorld.setZero();
				footTaskData[i].linAccWorld.setZero();
				// footTaskData[i].nextContactPos = estimatedState->footState[i].pos;
			}
			else
			{
				footSwingTrajectory[i].computeSwingTrajectoryBezier(swingPhase, gaitData->swingTime[i]);
				footTaskData[i].pos = x2leg_rotations[i] * footSwingTrajectory[i].getPosition();
				footTaskData[i].vWorld = x2leg_rotations[i] * footSwingTrajectory[i].getVelocity();
				footTaskData[i].linAccWorld = x2leg_rotations[i] * footSwingTrajectory[i].getAcceleration();
				// footTaskData[i].nextContactPos = x2leg_rotations[i] * footSwingTrajectory[i].getFinalPosition();
				// footTaskData[i].nextContactPos = planned_Pf[i];
			}
		}
		else // stance
		{
			// TODO
			footSwingTrajectory[i].setInitialPosition(x2leg_rotations[i].transpose() * estimatedState->footState[i].pos);
			bezier_init_Pf[i] = estimatedState->footState[i].pos;
			firstSwing[i] = true;
			footTaskData[i].pos = bezier_init_Pf[i];
			footTaskData[i].vWorld = Vec3::Zero();
			footTaskData[i].linAccWorld = Vec3::Zero();
			footTaskData[i].nextContactPos = planned_Pf[i];
		}

		Vec3 pHipWorld = estimatedState->floatingBaseState.pos + estimatedState->floatingBaseState.R_wb * pHipBody[i];
		if (footTaskData[i].pos.z() > pHipWorld.z() - 0.1)
		{
			footTaskData[i].pos.z() = pHipWorld.z() - 0.1;
			footTaskData[i].vWorld = Vec3::Zero();
			footTaskData[i].linAccWorld = Vec3::Zero();
		}
		else if ((pHipWorld - footTaskData[i].pos).norm() > 5.0)
		{
			footTaskData[i].pos = estimatedState->footState[i].pos;
			footTaskData[i].vWorld = Vec3::Zero();
			footTaskData[i].linAccWorld = Vec3::Zero();
		}

		memcpy(debug_info.pFootholds + 3 * i, planned_Pf[i].data(), sizeof(double) * 3);
		memcpy(debug_info.pFootholds_MRH + 3 * i, planned_Pf_MRH[i].data(), sizeof(double) * 3);

		std::cout << "foot des " << i << ": " << planned_Pf[i].transpose() << "\n";

		for (int n = 0; n < 50; n++)
		{
			footSwingTrajectory[i].computeSwingTrajectoryBezier(double(n) / 50, gaitData->swingTime[i]);
			Vec3 tmp_point = x2leg_rotations[i] * footSwingTrajectory[i].getPosition();
			memcpy(debug_info.pSwingTraj + 3 * 50 * i + 3 * n, tmp_point.data(), sizeof(double) * 3);
		}
	}

	debug_info.cmd_velocity = vBodyDes[0];
	debug_info.planned_velocity[0] = optimal_planned_variables[0];
	debug_info.planned_velocity[1] = optimal_planned_variables[2];
	debug_info.planned_velocity[2] = 0;
	debug_info.foot_planner_time = planer_time_cost;
	for (int i_leg = 0; i_leg < 4; i_leg++)
	{
		debug_info.selected_plane_id[i_leg] = optimal_plane_combination[i_leg];
		debug_info.considered_plane[i_leg].num = foot_planes_id[i_leg].size();
		debug_info.considered_plane[i_leg].id.clear();
		for (auto id : foot_planes_id[i_leg])
		{
			debug_info.considered_plane[i_leg].id.push_back(id);
		}
	}
	lcm.publish("FOOTHOLD_PLANNER_DUBUGINFO", &debug_info);
	iter++;
}
