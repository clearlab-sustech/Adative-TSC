#pragma once

#include <core/types.h>
#include <string>

using namespace std;

namespace clear {
class Task {
public:
  Task(string name);

  ~Task();

  Task operator+(const &Task task);

  matrix_t A;
  vector_t b;
  matrix_t C;
  vector_t lb;
  vector_t ub;
  string name_;
};

} // namespace clear
