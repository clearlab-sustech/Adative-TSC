#include "unitree_hw/UnitreeHW.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const char *filename = nullptr;
  if (argc > 1) {
    filename = argv[1];
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hardware"),
                 "config file for hardware is required.");
    throw std::runtime_error("no config file for hardware");
  }

  auto node_ptr = std::make_shared<clear::UnitreeHW>(filename);
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
