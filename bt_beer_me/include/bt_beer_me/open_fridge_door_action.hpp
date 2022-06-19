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

#include <bt_beer_me/action_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robot_impedance_controllers/action/follow_trajectory.hpp>

class OpenFridgeDoorAction : public ActionClient<robot_impedance_controllers::action::FollowTrajectory>
{
public:
  OpenFridgeDoorAction(const std::string& name, const BT::NodeConfiguration& config)
    : ActionClient<ActionType>(name, config)
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
    rclcpp::Node::SharedPtr node = getNode();
    if (!node)
    {
      return BT::NodeStatus::FAILURE;
    }

    auto logger = node->get_logger();

    // Get location of fridge
    geometry_msgs::msg::PoseStamped marker_pose;
    getInput("marker_pose", marker_pose);

    if (!connect("/arm/follow_trajectory_with_impedance"))
    {
      return BT::NodeStatus::FAILURE;
    }

    // Send the goal
    {
      ActionType::Goal goal;
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
      sendGoal(goal);
    }

    // Monitor for completion
    return waitForResult();
  }
};
