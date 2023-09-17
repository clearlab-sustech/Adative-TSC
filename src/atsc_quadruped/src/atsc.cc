#include "AtscImpl.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const char *filename = nullptr;
  if (argc > 1) {
    filename = argv[1];
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Estimator"),
                 "config file for estimator is required.");
    throw std::runtime_error("no config file for estimator");
  }
  auto node = std::make_shared<clear::AtscImpl>(filename);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
