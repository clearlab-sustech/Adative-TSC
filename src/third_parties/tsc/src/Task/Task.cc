

#include "tsc/Task/Task.h"

using namespace clear;

Task::Task(PinocchioInterface &robot, string name)
    : _robot(robot), _enable(true), _name(name) {
  n_var = robot.nv() + robot.na() + 3 * robot.nc();
}

const string &Task::name() { return _name; }

PinocchioInterface &Task::robot() { return _robot; }

bool Task::is_enable() { return _enable; }

void Task::enable() { _enable = true; }

void Task::disable() { _enable = false; }
