

#include "tsc/TaskSpaceControl.h"
#include <fstream>

using namespace clear;

TaskSpaceControl::TaskSpaceControl(PinocchioInterface &robot) : _robot(robot) {
  n_var_ = robot.nv() + robot.na() + 3 * robot.nc();
  H.resize(n_var_, n_var_);
  g.resize(n_var_);
  _sol.setZero(n_var_);
}

void TaskSpaceControl::addTask(std::shared_ptr<Task> task) {
  for (auto &t : _tasks) {
    if (t->name() == task->name()) {
      throw std::runtime_error(
          task->name() + " task has the same name with oneuin the task array");
    }
  }
  _tasks.push_back(task);
}

void TaskSpaceControl::addLinearConstraint(
    std::shared_ptr<LinearConstraints> constraints) {
  for (auto &c : _linearConstraints) {
    if (c->name() == constraints->name()) {
      throw std::runtime_error(
          constraints->name() +
          " task has the same name with oneuin the task array");
    }
  }
  _linearConstraints.push_back(constraints);
}

void TaskSpaceControl::solve() {
  H.setZero();
  g.setZero();
  for (auto &task : _tasks) {
    if (task->is_enable()) {
      task->update();
      if (task->H().rows() != n_var_ || task->H().cols() != n_var_ ||
          task->g().size() != n_var_) {
        printf("n_var = %ld, H().rows() = %ld, H().cols() = %ld, "
               "g.size() = %ld\n",
               n_var_, task->H().rows(), task->H().cols(), task->g().size());
        throw std::runtime_error(task->name() +
                                 " task  matrix (H,g) dimension is wrong");
      }
      H += task->H();
      g += task->g();
    }
  }

  size_t nDims_cstrs = 0;
  size_t nDims_cstrs_eq = 0;
  for (auto &cstr : _linearConstraints) {
    if (cstr->is_enable()) {
      cstr->update();
      if (cstr->isEqual()) {
        assert(cstr->c_lb() == cstr->c_ub());
        if (cstr->C().cols() != n_var_ ||
            cstr->C().rows() != cstr->c_ub().size() ||
            cstr->C().rows() != cstr->c_lb().size()) {
          printf("n_var = %ld, C().rows() = %ld, C().cols() = %ld, "
                 "c_ub().size() = %ld, cstr->c_lb().size() = %ld\n",
                 n_var_, cstr->C().rows(), cstr->C().cols(),
                 cstr->c_ub().size(), cstr->c_lb().size());
          throw runtime_error(
              cstr->name() +
              " constraints matrix (C, c_lb, c_ub) dimension is wrong");
        }
        nDims_cstrs_eq += cstr->C().rows();
      } else {
        if (cstr->C().cols() != n_var_ ||
            cstr->C().rows() != cstr->c_ub().size() ||
            cstr->C().rows() != cstr->c_lb().size()) {
          printf("n_var = %ld, C().rows() = %ld, C().cols() = %ld, "
                 "c_ub().size() = %ld, cstr->c_lb().size() = %ld\n",
                 n_var_, cstr->C().rows(), cstr->C().cols(),
                 cstr->c_ub().size(), cstr->c_lb().size());
          throw runtime_error(
              cstr->name() +
              " constraints matrix (C, c_lb, c_ub) dimension is wrong");
        }
        nDims_cstrs += cstr->C().rows();
      }
    }
  }
  C.setZero(nDims_cstrs, n_var_);
  c_lb.setZero(nDims_cstrs);
  c_ub.setZero(nDims_cstrs);

  Ce.setZero(nDims_cstrs_eq, n_var_);
  ce.setZero(nDims_cstrs_eq);

  size_t sr = 0;
  size_t sre = 0;
  for (size_t i = 0; i < _linearConstraints.size(); i++) {
    if (_linearConstraints[i]->is_enable()) {
      if (_linearConstraints[i]->isEqual()) {
        Ce.middleRows(sre, _linearConstraints[i]->C().rows()) =
            _linearConstraints[i]->C();
        ce.segment(sre, _linearConstraints[i]->c_lb().size()) =
            _linearConstraints[i]->c_lb();
        sre += _linearConstraints[i]->C().rows();
      } else {
        C.middleRows(sr, _linearConstraints[i]->C().rows()) =
            _linearConstraints[i]->C();
        c_lb.segment(sr, _linearConstraints[i]->c_lb().size()) =
            _linearConstraints[i]->c_lb();
        c_ub.segment(sr, _linearConstraints[i]->c_ub().size()) =
            _linearConstraints[i]->c_ub();
        sr += _linearConstraints[i]->C().rows();
      }
    }
  }

  QpSolver::DimsSpec dims;
  dims.nv = n_var_;
  dims.ne = nDims_cstrs_eq;
  dims.ng = nDims_cstrs;
  QpSolver::QpSolverSettings settings;
  settings.iter_max = 100;
  settings.verbose = false;

  auto solver_ptr = std::make_shared<QpSolver>(dims, settings);

  solver_ptr->update(H, g, Ce, ce, C, c_lb, c_ub);

  // saveAllData("tsc_debug.txt");

  _sol = solver_ptr->solve();

#ifdef PRINT_ERR
  printCstrsErr();
#endif
}

vector_t TaskSpaceControl::getOptimalQacc() { return _sol.head(_robot.nv()); }

vector_t TaskSpaceControl::getOptimalTorque() {
  return _sol.segment(_robot.nv(), _robot.na());
}

vector_t TaskSpaceControl::getOptimalContactForce() {
  return _sol.tail(3 * _robot.nc());
}

void TaskSpaceControl::activateTask(string name) {
  for (auto &task : _tasks) {
    if (task->name() == name) {
      task->enable();
      break;
    }
  }
}

void TaskSpaceControl::suspendTask(string name) {
  for (auto &task : _tasks) {
    if (task->name() == name) {
      task->disable();
      break;
    }
  }
}

void TaskSpaceControl::enableLinearConstraint(string name) {
  for (auto &cstr : _linearConstraints) {
    if (cstr->name() == name) {
      cstr->enable();
      break;
    }
  }
}

void TaskSpaceControl::disableLinearConstraint(string name) {
  for (auto &cstr : _linearConstraints) {
    if (cstr->name() == name) {
      cstr->disable();
      break;
    }
  }
}

void TaskSpaceControl::printCstrsErr() {
  for (auto &cstr : _linearConstraints) {
    cstr->errPrint(_sol);
  }
}

void TaskSpaceControl::saveAllData(string file_name) {
  ofstream outfile(file_name);
  if (!outfile.is_open()) {
    throw std::runtime_error(
        "[TaskSpaceControl::saveAllData] The file can not be opened");
  }
  outfile << "-----------------------H------------------------" << endl
          << H << endl
          << "-----------------------g------------------------" << endl
          << g.transpose() << endl
          << "-----------------------Ce------------------------" << endl
          << Ce << endl
          << "-----------------------c_e------------------------" << endl
          << ce.transpose() << endl
          << "-----------------------EqualityCstrs Err------------------------"
          << endl
          << (Ce * _sol).transpose() - ce.transpose() << endl
          << "-----------------------C------------------------" << endl
          << C << endl
          << "-----------------------c_lb------------------------" << endl
          << c_lb.transpose() << endl
          << "-----------------------c_ub------------------------" << endl
          << c_ub.transpose() << endl
          << "-----------------------sol------------------------" << endl
          << _sol.transpose() << endl
          << "-----------------------optimal_torque------------------------"
          << endl
          << getOptimalTorque().transpose() << endl
          << "-----------------------optimal_force------------------------"
          << endl
          << getOptimalContactForce().transpose() << endl
          << "-----------------------spring force------------------------"
          << endl;

  outfile.close();
}
