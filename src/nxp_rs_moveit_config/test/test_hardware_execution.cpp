// Exercise the installed JTC with in-memory joint interfaces. No hardware plugin
// or CAN device is loaded. The parameters are the files used by hardware_moveit.
#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using Action = control_msgs::action::FollowJointTrajectory;
using Handle = rclcpp_action::ClientGoalHandle<Action>;

class HardwareExecutionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    controller_ = std::make_shared<joint_trajectory_controller::JointTrajectoryController>();
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    options.arguments({"--ros-args", "--params-file",
      std::string(PACKAGE_SOURCE_DIR) + "/../rs_control/config/ros2_controllers.yaml",
      "--params-file", std::string(PACKAGE_SOURCE_DIR) + "/config/hardware_execution.yaml"});
    ASSERT_EQ(controller_->init("arm_controller", "", options), controller_interface::return_type::OK);
    ASSERT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    commands_.reserve(6);
    states_.reserve(12);
    for (size_t i = 0; i < 6; ++i) {
      const auto name = "joint-" + std::to_string(i + 1);
      commands_.emplace_back(name, "position", &command_[i]);
      states_.emplace_back(name, "position", &position_[i]);
      states_.emplace_back(name, "velocity", &velocity_[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (auto & command : commands_) { loaned_commands.emplace_back(command); }
    for (auto & state : states_) { loaned_states.emplace_back(state); }
    controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    ASSERT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    client_node_ = std::make_shared<rclcpp::Node>("execution_test_client");
    client_ = rclcpp_action::create_client<Action>(client_node_, "/arm_controller/follow_joint_trajectory");
    executor_->add_node(client_node_);
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    ASSERT_TRUE(client_->wait_for_action_server(2s));
  }

  void TearDown() override
  {
    if (controller_) {
      controller_->get_node()->deactivate();
      controller_->release_interfaces();
      executor_->remove_node(controller_->get_node()->get_node_base_interface());
    }
    if (client_node_) { executor_->remove_node(client_node_); }
    client_.reset();
    client_node_.reset();
    controller_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  void step()
  {
    // Deterministic plant: follow the last command with a selectable static bias.
    for (size_t i = 0; i < 6; ++i) { position_[i] = command_[i] + bias_[i]; }
    executor_->spin_some();
    controller_->update(controller_->get_node()->now(), rclcpp::Duration(10ms));
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }

  template<typename Future>
  bool wait(Future & future, std::chrono::seconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (future.wait_for(0ms) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() > deadline) { return false; }
      step();
    }
    return true;
  }

  std::shared_ptr<Handle> send(double final_velocity = 0.0)
  {
    Action::Goal goal;
    for (int i = 1; i <= 6; ++i) { goal.trajectory.joint_names.push_back("joint-" + std::to_string(i)); }
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(command_.begin(), command_.end());
    point.positions[1] += 0.02;
    point.velocities.assign(6, final_velocity);
    point.time_from_start = rclcpp::Duration(1s);
    goal.trajectory.points.push_back(point);
    auto future = client_->async_send_goal(goal);
    if (!wait(future)) { ADD_FAILURE() << "goal response timed out"; return nullptr; }
    return future.get();
  }

  std::array<double, 6> command_{}, position_{}, velocity_{}, bias_{};
  std::vector<hardware_interface::CommandInterface> commands_;
  std::vector<hardware_interface::StateInterface> states_;
  std::shared_ptr<joint_trajectory_controller::JointTrajectoryController> controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(HardwareExecutionTest, TwoAccurateTrajectoriesSucceed)
{
  for (int i = 0; i < 2; ++i) {
    auto handle = send();
    ASSERT_NE(handle, nullptr);
    auto result = client_->async_get_result(handle);
    ASSERT_TRUE(wait(result));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(result.get().result->error_code, Action::Result::SUCCESSFUL);
  }
}

TEST_F(HardwareExecutionTest, RecordedShoulderAndElbowErrorsAbort)
{
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  bias_[1] = 0.0663109018354574;
  bias_[3] = 0.08013886204025167;
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->error_code, Action::Result::PATH_TOLERANCE_VIOLATED);
}

TEST_F(HardwareExecutionTest, SmallPersistentErrorFailsGoalDeadline)
{
  // Within 0.03 path tolerance, outside 0.01 goal tolerance.
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  bias_[1] = 0.015;
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->error_code, Action::Result::GOAL_TOLERANCE_VIOLATED);
}

TEST_F(HardwareExecutionTest, NonzeroFinalVelocityRejected)
{
  EXPECT_EQ(send(0.1), nullptr);
}
