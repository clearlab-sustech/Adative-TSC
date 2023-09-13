#include "MeasuredState_lcm.hpp"
#include "JointsCmd_lcm.hpp"
#include "lcm/lcm-cpp.hpp"
#include "DataSets.h"
#include "PerceptionTerrain_lcm.hpp"
#include "PerceptionLocation_lcm.hpp"
#include "orientation_tools.h"

// #define REALROBOT_RUNNING

using namespace ori;

extern DataSets dataSets;
extern bool exitrequest;

namespace lcm_task
{
    // lcm message
    MeasuredState_lcm measuredState;
    JointsCmd_lcm jointsCmd;
    lcm::LCM lcm_pub;
    lcm::LCM lcm_rec;
    bool top_arrived = false;

    Timer tc_;

    class LcmSubHandler
    {
    public:
        ~LcmSubHandler() {}

        void handleMessage(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const MeasuredState_lcm *msg)
        {
            dataSets.measuredState.mtx.lock();
            std::memcpy(dataSets.measuredState.imuData.quat.data(), msg->imuData.quat, 4 * sizeof(double));
            std::memcpy(dataSets.measuredState.imuData.gyro.data(), msg->imuData.gyro, 3 * sizeof(double));
            std::memcpy(dataSets.measuredState.imuData.acc.data(), msg->imuData.acc, 3 * sizeof(double));
            std::memcpy(dataSets.measuredState.jointsState.qpos.data(), msg->jointsState.qpos, 12 * sizeof(double));
            std::memcpy(dataSets.measuredState.jointsState.qvel.data(), msg->jointsState.qvel, 12 * sizeof(double));
            std::memcpy(dataSets.measuredState.jointsState.tau.data(), msg->jointsState.tau, 12 * sizeof(double));
            std::memcpy(dataSets.measuredState.footForce, msg->force, 4 * sizeof(int16_t));
            /* for(int i=0; i < 4; i++)
            {
                printf("%d, ", msg->force[i]);
                printf("\n");
            } */
            dataSets.measuredState.isUpdated = true;
            dataSets.measuredState.mtx.unlock();
            //            printf("time: %f\n", tc_.getMs());
        }

        void handleTerrainMessage(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &chan,
                                  const PerceptionTerrain_lcm *msg)
        {
            dataSets.perceptionData.mtx.lock();

            memcpy(dataSets.perceptionData.pos.data(), msg->pos, 3 * sizeof(double));
            memcpy(dataSets.perceptionData.quat.data(), msg->quat, 4 * sizeof(double));
#ifdef REALROBOT_RUNNING
            Mat4 T_camera2body, T_camera2visual, T_visual2origin;

            T_camera2body.setIdentity();
            double theta = 51.4423 / 180.0 * M_PI;
            T_camera2body.topLeftCorner<3, 3>() << 0, sin(theta), -cos(theta),
                -1, 0, 0,
                0, cos(theta), sin(theta);
            T_camera2body.topRightCorner<3, 1>() << 0.3866, 0.0115, -0.03349;

            T_camera2visual.setIdentity();
            T_camera2visual.topLeftCorner<3, 3>() = quaternionToRotationMatrix(dataSets.location_vision.quat).transpose();
            T_camera2visual.topRightCorner<3, 1>() = dataSets.location_vision.pos;

            T_visual2origin.setIdentity();
            T_visual2origin.topLeftCorner<3, 3>() << 0, 0, -1,
                -1, 0, 0,
                0, 1, 0;
            T_visual2origin.topRightCorner<3, 1>() << 0.3866, 0.0115, 0.03651;

            Mat4 T_body2origin = T_visual2origin * T_camera2visual * T_camera2body.inverse();

            dataSets.perceptionData.pos = T_body2origin.topRightCorner<3, 1>();
            dataSets.perceptionData.quat = rotationMatrixToQuaternion(T_body2origin.topLeftCorner<3, 3>().transpose());
#endif
            for (int i = 0; i < msg->plane_num; i++)
            {
                memcpy(dataSets.perceptionData.planes[i].A.data(), msg->planes[i].A, 3 * sizeof(double));
                dataSets.perceptionData.planes[i].bound_num = msg->planes[i].bound_num;
                for (int n = 0; n < 6; n++)
                {
                    memcpy(dataSets.perceptionData.planes[i].Bs[n].data(), msg->planes[i].Bs[n], 3 * sizeof(double));
                    memcpy(dataSets.perceptionData.planes[i].Vertexs[n].data(), msg->planes[i].Vertexs[n],
                           2 * sizeof(double));
                }
            }
            for (int i = msg->plane_num; i < 30; i++)
            {
                memset(dataSets.perceptionData.planes[i].A.data(), 0, 3 * sizeof(double));
                dataSets.perceptionData.planes[i].bound_num = 0;
            }

            int initial_plane_i = msg->plane_num == 30 ? 29 : msg->plane_num;
            dataSets.perceptionData.planes[initial_plane_i].bound_num = 4;
            dataSets.perceptionData.planes[initial_plane_i].A << 0, 0, 0;
            dataSets.perceptionData.planes[initial_plane_i].Bs[0] = Eigen::Vector3d(0, 1, 1.0);
            dataSets.perceptionData.planes[initial_plane_i].Bs[1] = Eigen::Vector3d(1, 0, 0.65);
            dataSets.perceptionData.planes[initial_plane_i].Bs[2] = Eigen::Vector3d(0, -1, 1.0);
            dataSets.perceptionData.planes[initial_plane_i].Bs[3] = Eigen::Vector3d(-1, 0, 2);
            dataSets.perceptionData.planes[initial_plane_i].Vertexs[0] = Eigen::Vector2d(0.65, 1);
            dataSets.perceptionData.planes[initial_plane_i].Vertexs[1] = Eigen::Vector2d(0.65, -1);
            dataSets.perceptionData.planes[initial_plane_i].Vertexs[2] = Eigen::Vector2d(-2, -1);
            dataSets.perceptionData.planes[initial_plane_i].Vertexs[3] = Eigen::Vector2d(-2, 1);

            double final_plane_A3 = 0;
            double final_plane_x = 0;
            for (int i = 0; i < msg->plane_num; i++)
            {
                auto &plane_i = dataSets.perceptionData.planes[i];
                if (plane_i.bound_num == 0)
                    continue;
                double span_x = 0;
                for (int n = 1; n < plane_i.bound_num; n++)
                {
                    span_x = abs(plane_i.Vertexs[n - 1].x() - plane_i.Vertexs[n].x());
                    if (span_x > 0.5)
                    {
                        final_plane_A3 = plane_i.A.z();
                        final_plane_x = std::min(plane_i.Vertexs[n - 1].x(), plane_i.Vertexs[n].x());
                        top_arrived = true;
                        break;
                    }
                }
            }
            if (top_arrived)
            {
                int final_plane_i = initial_plane_i == 29 ? 28 : initial_plane_i + 1;
                dataSets.perceptionData.planes[final_plane_i].bound_num = 4;
                dataSets.perceptionData.planes[final_plane_i].A << 0, 0, final_plane_A3;
                dataSets.perceptionData.planes[final_plane_i].Bs[0] = Eigen::Vector3d(0, 1, 1.0);
                dataSets.perceptionData.planes[final_plane_i].Bs[1] = Eigen::Vector3d(1, 0, final_plane_x + 1);
                dataSets.perceptionData.planes[final_plane_i].Bs[2] = Eigen::Vector3d(0, -1, 1.0);
                dataSets.perceptionData.planes[final_plane_i].Bs[3] = Eigen::Vector3d(-1, 0, -final_plane_x);
                dataSets.perceptionData.planes[final_plane_i].Vertexs[0] = Eigen::Vector2d(final_plane_x + 1, 1);
                dataSets.perceptionData.planes[final_plane_i].Vertexs[1] = Eigen::Vector2d(final_plane_x + 1, -1);
                dataSets.perceptionData.planes[final_plane_i].Vertexs[2] = Eigen::Vector2d(final_plane_x, -1);
                dataSets.perceptionData.planes[final_plane_i].Vertexs[3] = Eigen::Vector2d(final_plane_x, 1);
            }

            for (int i = 0; i < msg->plane_num + 1; i++)
            {
                auto &plane_i = dataSets.perceptionData.planes[i];
                if (plane_i.bound_num == 0)
                    continue;
                double y_span1 = abs(plane_i.Vertexs[1].y() - plane_i.Vertexs[0].y());
                for (int n = 2; n < plane_i.bound_num; n++)
                {
                    if (abs(plane_i.Vertexs[n].y() - plane_i.Vertexs[n - 1].y()) > y_span1)
                    {
                        y_span1 = abs(plane_i.Vertexs[n].y() - plane_i.Vertexs[n - 1].y());
                    }
                }

                for (int j = i + 1; j < msg->plane_num + 1; j++)
                {
                    auto &plane_j = dataSets.perceptionData.planes[j];

                    if (plane_j.bound_num == 0)
                        continue;

                    if (abs(plane_i.A.z() - plane_j.A.z()) < 0.05)
                    {
                        double y_span2 = abs(plane_j.Vertexs[1].y() - plane_j.Vertexs[0].y());
                        for (int n = 2; n < plane_j.bound_num; n++)
                        {
                            if (abs(plane_j.Vertexs[n].y() - plane_j.Vertexs[n - 1].y()) > y_span2)
                            {
                                y_span2 = abs(plane_j.Vertexs[n].y() - plane_j.Vertexs[n - 1].y());
                            }
                        }
                        if (y_span1 >= y_span2)
                        {
                            plane_j.bound_num = 0;
                        }
                        else
                        {
                            plane_i.bound_num = 0;
                        }
                    }
                }
            }

            dataSets.perceptionData.mtx.unlock();
        }

        void handleLocationMessage(const lcm::ReceiveBuffer *rbuf,
                                   const std::string &chan,
                                   const PerceptionLocation_lcm *msg)
        {
            dataSets.location_vision.mtx.lock();
            memcpy(dataSets.location_vision.pos.data(), msg->pos, 3 * sizeof(double));
            memcpy(dataSets.location_vision.quat.data(), msg->quat, 4 * sizeof(double));

#ifdef REALROBOT_RUNNING
            Mat4 T_camera2body, T_camera2visual, T_visual2origin;

            T_camera2body.setIdentity();
            double theta = 51.4423 / 180.0 * M_PI;
            T_camera2body.topLeftCorner<3, 3>() << 0, sin(theta), -cos(theta),
                -1, 0, 0,
                0, cos(theta), sin(theta);
            T_camera2body.topRightCorner<3, 1>() << 0.3866, 0.0115, -0.03349;

            T_camera2visual.setIdentity();
            T_camera2visual.topLeftCorner<3, 3>() = quaternionToRotationMatrix(dataSets.location_vision.quat).transpose();
            T_camera2visual.topRightCorner<3, 1>() = dataSets.location_vision.pos;

            T_visual2origin.setIdentity();
            T_visual2origin.topLeftCorner<3, 3>() << 0, 0, -1,
                -1, 0, 0,
                0, 1, 0;
            T_visual2origin.topRightCorner<3, 1>() << 0.3866, 0.0115, 0.03651;

            Mat4 T_body2origin = T_visual2origin * T_camera2visual * T_camera2body.inverse();

            dataSets.location_vision.pos = T_body2origin.topRightCorner<3, 1>();
            dataSets.location_vision.quat.head(3) = rotationMatrixToRPY(T_body2origin.topLeftCorner<3, 3>().transpose());
#endif
            dataSets.location_vision.isUpdated = true;
            dataSets.location_vision.mtx.unlock();
        }
    };

    void subscribe()
    {
        while (lcm_rec.handleTimeout(1) >= 0 && !exitrequest)
            ;
        printf("shutdown lcm subscriber\n");
    }

    void publishJointsCmd()
    {
        if (!exitrequest)
        {
            dataSets.jointsCmd.mtx.lock();
            std::memcpy(jointsCmd.tau_ff, dataSets.jointsCmd.tau_ff.data(), 12 * sizeof(double));
            std::memcpy(jointsCmd.qpos_des, dataSets.jointsCmd.qpos_des.data(), 12 * sizeof(double));
            std::memcpy(jointsCmd.qvel_des, dataSets.jointsCmd.qvel_des.data(), 12 * sizeof(double));
            std::memcpy(jointsCmd.Kp, dataSets.jointsCmd.Kp.data(), 12 * sizeof(double));
            std::memcpy(jointsCmd.Kd, dataSets.jointsCmd.Kd.data(), 12 * sizeof(double));
            dataSets.jointsCmd.mtx.unlock();
            if (lcm_pub.publish("AliengoJointsCmd", &jointsCmd) != 0)
            {
                printf("lcm failed in publishing measuredState\n");
            }
        }
    }
}