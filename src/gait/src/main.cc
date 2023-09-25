#include "gait/GaitSchedule.h"
#include <rclcpp/rclcpp.hpp>

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
  auto node = std::make_shared<clear::GaitSchedule>(filename);

  rclcpp::spin(node);

  rclcpp::shutdown();
}