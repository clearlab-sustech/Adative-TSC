#ifndef TASK_SPACE_CONTROL_TASK_H
#define TASK_SPACE_CONTROL_TASK_H

#include <core/types.h>
#include <pinocchio/PinocchioInterface.h>

namespace clear {
class Task {
public:
  explicit Task(PinocchioInterface &robot, string name);

  virtual void update() = 0;

  virtual const matrix_t &H() = 0;

  virtual const vector_t &g() = 0;

  virtual const string &name();

  virtual bool is_enable();

  virtual void enable();

  virtual void disable();

  PinocchioInterface &robot();

protected:
  PinocchioInterface &_robot;
  size_t n_var;
  bool _enable;

private:
  string _name;
};
} // namespace clear

#endif // TASK_SPACE_CONTROL_TASK_H
