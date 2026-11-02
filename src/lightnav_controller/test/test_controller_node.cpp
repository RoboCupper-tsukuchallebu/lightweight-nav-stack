#include <gtest/gtest.h>

#include <memory>

#include "lightnav_controller/controller_node.hpp"
#include "rclcpp/rclcpp.hpp"


class ControllerNodeLifecycle : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { node_ = std::make_shared<lightnav_controller::ControllerNode>(); }

  void configureAndActivate() {
    ASSERT_EQ(node_->configure().label(), "inactive");
    ASSERT_EQ(node_->activate().label(), "active");
  }

  std::shared_ptr<lightnav_controller::ControllerNode> node_;
};

TEST_F(ControllerNodeLifecycle, ShutdownFromActiveStopsTimerAndOutputs) {
  configureAndActivate();
  ASSERT_FALSE(node_->isReleased());
  EXPECT_EQ(node_->shutdown().label(), "finalized");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, ShutdownFromInactiveReleasesOutputs) {
  ASSERT_EQ(node_->configure().label(), "inactive");
  ASSERT_FALSE(node_->isReleased());
  EXPECT_EQ(node_->shutdown().label(), "finalized");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, ShutdownFromUnconfiguredFinalizes) {
  EXPECT_EQ(node_->shutdown().label(), "finalized");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, DeactivateStopsTimer) {
  configureAndActivate();
  ASSERT_EQ(node_->deactivate().label(), "inactive");
  EXPECT_FALSE(node_->isTimerRunning());
}

TEST_F(ControllerNodeLifecycle, CleanupReleasesOutputs) {
  configureAndActivate();
  node_->deactivate();
  ASSERT_EQ(node_->cleanup().label(), "unconfigured");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, ConfigureFailsOnNonPositiveParameter) {
  node_->set_parameter(rclcpp::Parameter("v_max", -1.0));
  EXPECT_EQ(node_->configure().label(), "unconfigured");
}
