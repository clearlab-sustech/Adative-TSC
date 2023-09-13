#ifndef XIAOTIANHYBRID_PERCEPTIONDATA_H
#define XIAOTIANHYBRID_PERCEPTIONDATA_H

#include "cppTypes.h"
#include <mutex>

struct Plane
{
    Vec3 A;
    int bound_num;
    std::array<Vec3, 6> Bs;
    std::array<Vec2, 6> Vertexs;
    Vec3 center;
};

struct Location
{
    std::mutex mtx;
    Vec3 pos;
    Quat quat;
    bool isUpdated = false;
};

struct PerceptionData
{
    std::mutex mtx;
    std::array<Plane, 30> planes;
    Vec3 pos;
    Quat quat;
};

#endif //XIAOTIANHYBRID_PERCEPTIONDATA_H