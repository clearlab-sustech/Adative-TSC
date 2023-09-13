//
// Created by wenchun on 3/22/21.
//

#ifndef XIAOTIANHYBRID_GAIT_H
#define XIAOTIANHYBRID_GAIT_H

#include "cppTypes.h"

using Eigen::Array4d;
using Eigen::Array4i;

class Gait {
public:
  Gait(int nSegment, Vec4Int offset, Vec4Int durations, const std::string &name);

  ~Gait();

  Vec4 getContactState();

  Vec4 getSwingState();

  int *getMpcTable();

  void run(int iterationsBetweenMPC, int currentIteration);

  Vec4 getStanceTime(double dtMPC) const;

  Vec4 getSwingTime(double dtMPC) const;

  int getCurrentGaitPhase() const;

  double getCurrentPhaseDouble() const;

  void debugPrint();

private:
  std::string _name;
  int *_mpc_table;
  Array4i _offsets;   // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4d _offsetsDouble; // offsets in phase (0 to 1)
  Array4d _durationsDouble; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  double _phase;

  Array4d swingPhase;
  Array4d stancePhase;
};

#endif //XIAOTIANHYBRID_GAIT_H
