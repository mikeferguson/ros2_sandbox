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

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_impedance_controllers/action/follow_trajectory.hpp>

class OpenFridgeDoorAction : public BT::AsyncActionNode
{
  using FollowTrajectory = robot_impedance_controllers::action::FollowTrajectory;
  using FollowTrajectoryGoal =  robot_impedance_controllers::action::FollowTrajectory::Goal;
  using FollowTrajectoryResult =  robot_impedance_controllers::action::FollowTrajectory::Result::SharedPtr;
  using FollowTrajectoryWrappedResult =
    rclcpp_action::ClientGoalHandle<robot_impedance_controllers::action::FollowTrajectory>::WrappedResult;

  enum class State
  {
    ACTIVE,
    SUCCEEDED,
    FAILED,
  };

public:
  OpenFridgeDoorAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    // Perception returns the location of an AR marker
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("marker_pose") };
  }

  BT::NodeStatus tick() override
  {
    // Get node for ROS2 interfaces
    rclcpp::Node::SharedPtr node =
      config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    if (!node)
    {
      return BT::NodeStatus::FAILURE;
    }

    auto logger = node->get_logger();

    // Get location of fridge
    geometry_msgs::msg::PoseStamped marker_pose;
    getInput("marker_pose", marker_pose);

    if (!client_)
    {
      // Need to create action client
      client_ = rclcpp_action::create_client<FollowTrajectory>(node,
        "/arm/follow_trajectory_with_impedance");
      if (!client_->wait_for_action_server(std::chrono::seconds(1)))
      {
        RCLCPP_ERROR(logger, "Timed out connecting to /arm/follow_trajectory_with_impedance");
        return BT::NodeStatus::FAILURE;
      }
    }

    // Send the goal
    {
      FollowTrajectory::Goal goal;
      goal.header.frame_id = "base_link";
      // goal.trajectory = 
      goal.goal_tolerance.linear.x = 0.1;
      goal.goal_tolerance.linear.y = 0.1;
      goal.goal_tolerance.linear.z = 0.1;
      goal.goal_tolerance.angular.x = 0.1;
      goal.goal_tolerance.angular.y = 0.1;
      goal.goal_tolerance.angular.z = 0.1;
      goal.use_pose_tolerance = false;
      goal.use_twist_tolerance = false;

      // Trajectory looks like:
      //   pose is arc (could even be a line at an angle?)
      //     z height is very stiff
      //     some tolerance in x/y pose
      //     pitch/roll are very stiff
      //     yaw is free (no gains)
      //   twist is all 0 (gains are all 0)
      //   wrench is all 0

      // Setup callbacks
      auto goal_options = typename rclcpp_action::Client<FollowTrajectory>::SendGoalOptions();
      goal_options.result_callback =
        std::bind(&OpenFridgeDoorAction::resultCallback, this, std::placeholders::_1);

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
  void resultCallback(const FollowTrajectoryWrappedResult& result)
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

  std::shared_ptr<rclcpp_action::Client<FollowTrajectory>> client_;
  FollowTrajectoryResult result_;
  State goal_state_;
};
