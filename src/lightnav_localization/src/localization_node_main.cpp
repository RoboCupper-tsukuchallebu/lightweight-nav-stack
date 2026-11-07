#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lightnav_localization/localization_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lightnav_localization::LocalizationNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
