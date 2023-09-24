#ifndef TASK_SPACE_CONTROL_CONSTRAINTS_H
#define TASK_SPACE_CONTROL_CONSTRAINTS_H

#include <core/types.h>
#include <pinocchio/PinocchioInterface.h>

namespace clear {
class LinearConstraints {
public:
  explicit LinearConstraints(PinocchioInterface &robot, string name,
                             bool isEqual = false);

  virtual void update() = 0;

  virtual const matrix_t &C() = 0;

  virtual const vector_t &c_lb() = 0;

  virtual const vector_t &c_ub() = 0;

  virtual const string &name();

  virtual bool isEqual();

  virtual void errPrint(vector_t &u);

  virtual bool is_enable();

  virtual void enable();

  virtual void disable();

  PinocchioInterface &robot();

protected:
  PinocchioInterface &_robot;
  bool _isEqual, _enable;
  size_t n_var;

private:
  string _name;
};
} // namespace clear

#endif // TASK_SPACE_CONTROL_CONSTRAINTS_H
