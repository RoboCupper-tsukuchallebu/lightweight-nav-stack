#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lightnav_localization/localization_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lightnav_localization::LocalizationNode>();
  auto context = node->get_node_base_interface()->get_context();
  const auto pre_shutdown_handle = context->add_pre_shutdown_callback([node]() {
    if (node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      node->shutdown();
    }
  });
  rclcpp::spin(node->get_node_base_interface());
  if (node->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    node->shutdown();
  }
  context->remove_pre_shutdown_callback(pre_shutdown_handle);
  rclcpp::shutdown();
  return 0;
}
