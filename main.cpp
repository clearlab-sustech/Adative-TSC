#include <iostream>
#include <thread>
#include <fstream>
#include <csignal>
#include "DataSets.h"
#include "Controller.h"
#include "Estimator.h"
#include "GaitScheduler.h"
#include "Planner.h"
#include "Param.h"
#include "Configuration.h"
#include "Timer.h"
#include "lcm_task.h"
#include "PeriodicTask.h"
#include "ToRobot.h"
#include "UserInterface.h"

// #define SAVE_DATA
#define SAVE_PLANE
DataSets dataSets;

#ifdef REALROBOT_RUNNING
RealRobot *realRobot;
#endif

bool timeForCtrl = false;

bool exitrequest = false;

static void default_handler(int sig)
{
    (void)sig;
    exitrequest = true;
}

void loop_peroid()
{
    timeForCtrl = true;
}

int main()
{

    signal(SIGINT, default_handler);
    signal(SIGTSTP, default_handler);
    signal(SIGQUIT, default_handler);

    if (!lcm_task::lcm_pub.good() && !lcm_task::lcm_rec.good())
        return 1;
    std::thread lcm_rec_thread(lcm_task::subscribe);
    lcm_task::LcmSubHandler handler;
#ifndef REALROBOT_RUNNING
    lcm_task::lcm_rec.subscribe("AliengoState", &lcm_task::LcmSubHandler::handleMessage, &handler);
#endif
    lcm_task::lcm_rec.subscribe("PerceptionTerrain", &lcm_task::LcmSubHandler::handleTerrainMessage, &handler);
    lcm_task::lcm_rec.subscribe("PerceptionLocation", &lcm_task::LcmSubHandler::handleLocationMessage, &handler);

    PeriodicTaskManager taskManager;

#ifdef SAVE_DATA
    std::fstream save_state(THIS_COM "log/state.txt", std::ios::ate | std::ios::out);
    std::fstream save_cmd(THIS_COM "log/cmd.txt", std::ios::ate | std::ios::out);
    if (!save_state.is_open() || !save_cmd.is_open())
    {
        throw runtime_error("file open failed");
    }
#endif

    UserParameterHandler param(std::string(THIS_COM) + "/config/param/param.yaml");
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(),
                          dataSets.robotModelData.model);
    for (pin::JointIndex joint_id = 0; joint_id < (pin::JointIndex)dataSets.robotModelData.model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left
                  << dataSets.robotModelData.model.names[joint_id] << ":"
                  << dataSets.robotModelData.model.getJointId(dataSets.robotModelData.model.names[joint_id]) << endl;
    dataSets.robotModelData.data = pin::Data(dataSets.robotModelData.model);
    dataSets.robotModelData.flwID = dataSets.robotModelData.model.getBodyId(param.FL_WHEEL);
    dataSets.robotModelData.frwID = dataSets.robotModelData.model.getBodyId(param.FR_WHEEL);
    dataSets.robotModelData.hlwID = dataSets.robotModelData.model.getBodyId(param.HL_WHEEL);
    dataSets.robotModelData.hrwID = dataSets.robotModelData.model.getBodyId(param.HR_WHEEL);
    dataSets.robotModelData.flhID = dataSets.robotModelData.model.getJointId(param.FL_HIP);
    dataSets.robotModelData.frhID = dataSets.robotModelData.model.getJointId(param.FR_HIP);
    dataSets.robotModelData.hlhID = dataSets.robotModelData.model.getJointId(param.HL_HIP);
    dataSets.robotModelData.hrhID = dataSets.robotModelData.model.getJointId(param.HR_HIP);
    dataSets.robotModelData.baseID = dataSets.robotModelData.model.getBodyId(param.BASE);

#ifndef REALROBOT_RUNNING
    PeriodicFunction loop_lcm_pub(&taskManager, param.dt, "loop_lcm_pub_jointsCmd", lcm_task::publishJointsCmd);
    loop_lcm_pub.start();
#endif

    PeriodicFunction loop_peroid_func(&taskManager, param.dt, "loop_peroid", loop_peroid);
    loop_peroid_func.start();

#ifdef REALROBOT_RUNNING
    std::cout << "Control level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    realRobot = new RealRobot(UNITREE_LEGGED_SDK::LOWLEVEL, &dataSets.measuredState, &dataSets.jointsCmd);
    UNITREE_LEGGED_SDK::LoopFunc loop_control("control_loop", realRobot->dt, boost::bind(&RealRobot::RobotControl, realRobot));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpSend("udp_send", realRobot->dt, 3, boost::bind(&RealRobot::UDPSend, realRobot));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv("udp_recv", realRobot->dt, 3, boost::bind(&RealRobot::UDPRecv, realRobot));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
#endif

    Estimator estimator(&dataSets.measuredState,
                        &dataSets.robotModelData,
                        &dataSets.estimatedState,
                        &dataSets.perceptionData,
                        &dataSets.location_vision,
                        &param);

    GaitScheduler gait(&dataSets.estimatedState,
                       &dataSets.userCmd,
                       &dataSets.gaitData,
                       &param);

    Planner planner(&dataSets.estimatedState,
                    &dataSets.robotModelData,
                    &dataSets.userCmd,
                    &dataSets.gaitData,
                    &dataSets.tasksData,
                    &dataSets.perceptionData,
                    &param);

    Controller controller(&dataSets.estimatedState,
                          &dataSets.robotModelData,
                          &dataSets.tasksData,
                          &dataSets.gaitData,
                          &dataSets.jointsCmd,
                          &param);

    UserInterface userInterface(&param, &dataSets.userCmd);

    size_t iter = 0;
    Timer t;
    // std::cin.ignore();
    while (!exitrequest)
    {
        usleep(10);
        if (timeForCtrl)
        {
            // std::cout << "start control loop!" << std::endl;

            double time_control_start;
            time_control_start = t.getMs();
            timeForCtrl = false;

#ifdef REALROBOT_RUNNING
            // if (!dataSets.measuredState.isUpdated)
            realRobot->updateStateData();
#endif

            estimator.estimate();
            userInterface.update(&dataSets.estimatedState);

            gait.step();
            Vec4 contactPhaseDes;
            contactPhaseDes.array() =
                1 - dataSets.gaitData.stanceTimeRemain.array() / dataSets.gaitData.nextStanceTime.array();
            estimator.setContactPhase(contactPhaseDes); // TODO

            planner.plan();
            controller.run();
#ifdef REALROBOT_RUNNING
            // if (!dataSets.measuredState.isUpdated)
            realRobot->updateCmdData();
#endif
            double time_control_end;
            time_control_end = t.getMs();

#ifdef SAVE_DATA
            save_state
                << std::setprecision(9) << time_control_start << " " << time_control_end << " "
                << planner.time_mpc_solved << " "
                << dataSets.estimatedState.jointsState.qpos.transpose() << " "
                << dataSets.estimatedState.jointsState.qvel.transpose() << " "
                << dataSets.estimatedState.jointsState.tau.transpose() << " "
                << dataSets.estimatedState.floatingBaseState.pos.transpose() << " "
                << dataSets.estimatedState.floatingBaseState.rpy.transpose() << " "
                << dataSets.estimatedState.floatingBaseState.vWorld.transpose() << " "
                << dataSets.estimatedState.floatingBaseState.omegaWorld.transpose() << " ";
            for (int i = 0; i < 4; i++)
            {
                save_state
                    << dataSets.estimatedState.footState[i].pos.transpose() << " "
                    << dataSets.estimatedState.footState[i].vWorld.transpose() << " ";
            }
            save_state
                << dataSets.measuredState.imuData.acc.transpose() << " "
                << dataSets.measuredState.imuData.gyro.transpose() << " "
                << dataSets.measuredState.imuData.quat.transpose() << " ";
            for (int i = 0; i < 4; i++)
            {
                save_state
                    << dataSets.estimatedState.footState[i].force << " ";
                save_state
                    << dataSets.estimatedState.footState[i].contact_detected << " ";
                // save_state
                //     << dataSets.gaitData.late_contact[i] << " ";
                save_state
                    << dataSets.gaitData.stanceTimeRemain[i] << " ";
            }
            dataSets.location_vision.mtx.lock();
            save_state
                << dataSets.location_vision.pos.transpose() << " "
                << dataSets.location_vision.quat.transpose() << " ";
            dataSets.location_vision.mtx.unlock();

            save_state << std::endl;
            save_cmd << dataSets.jointsCmd.qpos_des.transpose() << " "
                     << dataSets.jointsCmd.qvel_des.transpose() << " "
                     << dataSets.jointsCmd.tau_ff.transpose() << " "
                     << dataSets.tasksData.forcesTaskData.forces_ref.transpose() << " "
                     << dataSets.tasksData.comTaskData.pos.transpose() << " "
                     << dataSets.tasksData.comTaskData.rpy.transpose() << " "
                     << dataSets.tasksData.comTaskData.vWorld.transpose() << " "
                     << dataSets.tasksData.comTaskData.omegaWorld.transpose() << " "
                     << dataSets.tasksData.comTaskData.footPlannedVBody.transpose() << " "
                     << dataSets.userCmd.vx_des << " " << dataSets.userCmd.vy_des << " "
                     << dataSets.userCmd.yawd_des << " ";
            for (int i = 0; i < 4; i++)
            {
                save_cmd
                    << dataSets.tasksData.footTaskData[i].pos.transpose() << " "
                    << dataSets.tasksData.footTaskData[i].vWorld.transpose() << " "
                    << dataSets.tasksData.footTaskData[i].linAccWorld.transpose() << " "
                    << dataSets.tasksData.footTaskData[i].nextContactPos.transpose() << " ";
            }
            save_cmd << std::endl;
#endif

            if (param.ctrl_num != 10)
            {
                iter++;
            }
        }
    }

#ifdef SAVE_DATA
    save_cmd.close();
    save_state.close();
#endif

    // lcm_rec_thread.join();
    return 0;
}
