#include "visualization/VisualizationNode.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp::executors;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const char *filename = nullptr;
  if (argc > 1) {
    filename = argv[1];
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("VisualizationNode"),
                 "config file for control is required.");
    throw std::runtime_error("no config file for control");
  }

  auto node = std::make_shared<clear::VisualizationNode>(filename);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
