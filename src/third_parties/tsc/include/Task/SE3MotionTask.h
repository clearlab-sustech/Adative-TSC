

#ifndef TASKSPACECONTROL_SE3MOTIONTASK_H
#define TASKSPACECONTROL_SE3MOTIONTASK_H

#include "Task/Task.h"

namespace clear {
class SE3MotionTask : public Task {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3MotionTask(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &H();

  virtual const vector_t &g();

  matrix6_t &Kp();

  matrix6_t &Kd();

  pin::SE3 &SE3Ref();

  vector6_t &spatialVelRef();

  vector6_t &spatialAccRef();

  scalar_t cost(vector_t &optimal_u);

  vector6_t error(vector_t &optimal_u);

  matrix6_t &weightMatrix();

private:
  matrix_t _H, A;
  matrix6x_t J;
  matrix6_t _Kp, _Kd, _Q;
  vector_t _g, a;
  pin::SE3 _SE3Ref, _SE3;
  vector6_t _spatialVel, _spatialVelRef, _spatialAccRef,
      acc_fb; // acc_fb , feedback term
  pin::FrameIndex frame_index;
};
} // namespace clear

#endif // TASKSPACECONTROL_SE3MOTIONTASK_H
