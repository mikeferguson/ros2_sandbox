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
#include <bt_beer_me/action_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Nav2Action : public ActionClient<nav2_msgs::action::NavigateToPose>
{
public:
  Nav2Action(const std::string& name, const BT::NodeConfiguration& config)
    : ActionClient<nav2_msgs::action::NavigateToPose>(name, config)
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

    if (!connect("navigate_to_pose"))
    {
      return BT::NodeStatus::FAILURE;
    }

    // Send the goal
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose = goal_pose;
    sendGoal(goal);

    // Monitor for completion
    return waitForResult();
  }
};
