#include "less/Task.h"

namespace clear {

Task::Task(string name) : name_(name) {}

Task::~Task() {}

Task Task::operator+(Task const &task) {
  Task res(this->name_ + task.name_);
  return res;
}

} // namespace clear
