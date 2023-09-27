#include "MotionManager.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

using namespace rclcpp::executors;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const char *filename = nullptr;
  if (argc > 1) {
    filename = argv[1];
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("atsc"),
                 "config file for atsc is required.");
    throw std::runtime_error("no config file for atsc");
  }
  auto node = std::make_shared<clear::MotionManager>(filename);

  MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

  try {
    node->init();
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what() << '\n');
  }

  rclcpp::shutdown();
}