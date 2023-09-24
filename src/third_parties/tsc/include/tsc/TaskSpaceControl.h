#ifndef TASK_SPACE_CONTROL_TASKSPACECONTROL_H
#define TASK_SPACE_CONTROL_TASKSPACECONTROL_H

#include "tsc/Constraints/LinearConstraints.h"
#include "tsc/Task/Task.h"
#include <core/types.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>
#include <qpsolver/QpSolver.h>

/* QP-based Controller */
/* Variables x=[\qacc, \torque, \force] */

namespace clear {
class TaskSpaceControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TaskSpaceControl(PinocchioInterface &robot);

  void addTask(std::shared_ptr<Task> task);

  void activateTask(string name);

  void suspendTask(string name);

  void addLinearConstraint(std::shared_ptr<LinearConstraints> constraints);

  void enableLinearConstraint(string name);

  void disableLinearConstraint(string name);

  void solve();

  vector_t getOptimalQacc(); // [qacc;f]

  vector_t getOptimalTorque(); // tau

  vector_t getOptimalContactForce(); // f

  void saveAllData(string file_name);

  void printCstrsErr();

private:
  PinocchioInterface &_robot;
  std::vector<std::shared_ptr<Task>> _tasks;
  std::vector<std::shared_ptr<LinearConstraints>> _linearConstraints;

  matrix_t H, C, Ce;
  vector_t g, c_lb, c_ub, ce, _sol;
  size_t n_var_;
};
} // namespace clear

#endif // TASK_SPACE_CONTROL_TASKSPACECONTROL_H
