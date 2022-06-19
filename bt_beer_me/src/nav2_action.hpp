/*
 * Copyright (C) 2022 Michael Ferguson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <memory>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_singletons.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Nav2Action : public BT::AsyncActionNode
{
  using Nav2Pose = nav2_msgs::action::NavigateToPose;
  using Nav2PoseGoal =  nav2_msgs::action::NavigateToPose::Goal;
  using Nav2PoseResult =  nav2_msgs::action::NavigateToPose::Result::SharedPtr;
  using Nav2PoseWrappedResult =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult;

  enum class State
  {
    ACTIVE,
    SUCCEEDED,
    FAILED,
  };

public:
  Nav2Action(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
  }

  BT::NodeStatus tick() override
  {
    // Get node for ROS2 interfaces
    auto node = getNode();

    if (!node)
    {
      return BT::NodeStatus::FAILURE;
    }

    auto logger = node->get_logger();

    // Get the navigation goal
    geometry_msgs::msg::PoseStamped goal_pose;
    getInput("goal", goal_pose);
    double yaw = tf2::getYaw(goal_pose.pose.orientation);
    RCLCPP_INFO_STREAM(logger, "Navigating to " << goal_pose.pose.position.x << ", " <<
                                                   goal_pose.pose.position.y << ", " <<
                                                   yaw);

    if (!client_)
    {
      // Need to create action client
      client_ = getClient();
      if (!client_->wait_for_action_server(std::chrono::seconds(1)))
      {
        RCLCPP_ERROR(logger, "Timed out connecting to /navigate_to_pose");
        return BT::NodeStatus::FAILURE;
      }
    }

    // Send the goal
    {
      Nav2Pose::Goal goal;
      goal.pose = goal_pose;

      // Setup callbacks
      auto goal_options = typename rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
      goal_options.result_callback =
        std::bind(&Nav2Action::resultCallback, this, std::placeholders::_1);

      goal_state_ = State::ACTIVE;
      client_->async_send_goal(goal, goal_options);
    }

    // Monitor for completion
    while (rclcpp::ok())
    {
      if (goal_state_ == State::SUCCEEDED)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else if (goal_state_ == State::FAILED)
      {
        return BT::NodeStatus::FAILURE;
      }
    }

    // Shutting down
    return BT::NodeStatus::FAILURE;
  }

private:
  void resultCallback(const Nav2PoseWrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      goal_state_ = State::SUCCEEDED;
    }
    else
    {
      goal_state_ = State::FAILED;
    }
    result_ = result.result;
  }

  static std::shared_ptr<rclcpp_action::Client<Nav2Pose>> getClient()
  {
    static std::shared_ptr<rclcpp_action::Client<Nav2Pose>> client;
    auto node = getNode();
    if (!client)
    {
      client = rclcpp_action::create_client<Nav2Pose>(node, "/navigate_to_pose");
    }
    return client;
  }

  std::shared_ptr<rclcpp_action::Client<Nav2Pose>> client_;
  Nav2PoseResult result_;
  State goal_state_;
};
