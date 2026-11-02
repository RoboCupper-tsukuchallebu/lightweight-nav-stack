#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lightnav_obstacle_monitor/obstacle_monitor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lightnav::ObstacleMonitorNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
