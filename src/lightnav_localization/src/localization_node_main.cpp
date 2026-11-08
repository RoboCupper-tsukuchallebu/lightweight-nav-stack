#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lightnav_localization/localization_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lightnav_localization::LocalizationNode>();
  auto context = node->get_node_base_interface()->get_context();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  std::promise<void> save_completed_promise;
  std::shared_future<void> save_completed = save_completed_promise.get_future().share();
  std::atomic<bool> shutdown_requested{false};

  // The SIGINT path runs rclcpp::shutdown() from the signal-handler thread. Cancel spin there,
  // then wait until the main thread has saved the map before the context is actually shut down.
  // All lifecycle work (batch optimization, localizer release) happens on the main thread.
  const auto pre_shutdown_handle = context->add_pre_shutdown_callback(
      [&executor, save_completed, &shutdown_requested]() {
        shutdown_requested.store(true);
        executor.cancel();
        save_completed.wait();
      });

  executor.spin();
  if (node->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    node->shutdown();
  }
  save_completed_promise.set_value();

  if (shutdown_requested.load()) {
    // The signal handler is finishing context shutdown after the promise above.
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  } else {
    rclcpp::shutdown();
  }
  context->remove_pre_shutdown_callback(pre_shutdown_handle);
  return 0;
}
