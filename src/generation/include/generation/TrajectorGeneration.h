#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;
using namespace std;

namespace clear {
class TrajectorGeneration : public Node {

public:
  TrajectorGeneration(string config_yaml);
  
  ~TrajectorGeneration();

private:
  /* data */
};

} // namespace clear
