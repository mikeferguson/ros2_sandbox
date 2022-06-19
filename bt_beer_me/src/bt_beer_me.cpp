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

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <nav2_action.hpp>
#include <open_fridge_door_action.hpp>
#include <ros2_singletons.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = getNode();

  std::string behavior_tree_xml =
    node->declare_parameter("behavior_tree_xml", std::string("/home/fergs/humble/workspace/src/ros2_sandbox/bt_beer_me/behavior_trees/beer_me.xml"));
  RCLCPP_INFO(node->get_logger(), "Loading XML : %s", behavior_tree_xml.c_str());

  // Setup components of behavior tree
  BT::BehaviorTreeFactory factory;
  // This calls navigation2 to get to fridge and delivery poses
  factory.registerNodeType<Nav2Action>("Nav2Action");
  // This calls moveit2 to get arm into pre grasp pose, grab the beer, etc
  //factory.registerNodeType<MoveitAction>("MoveitAction");
  // This calls the impedance controller to open the fridge door
  factory.registerNodeType<OpenFridgeDoorAction>("OpenFridgeDoorAction");
  // This executes perception to find the the beer pose
  //factory.registerNodeType<BeerPerceptionAction>("BeerPerceptionAction");
  // This calls the impedance controller to close the fridge door
  //factory.registerNodeType<ImpedanceTrajectoryCloseDoorAction>("ImpedanceTrajectoryCloseDoorAction");

  // Create the behavior tree
  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromFile(behavior_tree_xml, blackboard);

  // Hacky way to get access to node interface
  blackboard->set("node", node);

  // Set the default fridge location
  geometry_msgs::msg::PoseStamped fridge;
  fridge.header.frame_id = "map";
  fridge.pose.position.x = 1.0;
  fridge.pose.position.y = 2.0;
  fridge.pose.orientation.z = 0.707;
  fridge.pose.orientation.w = 0.707;
  blackboard->set("fridge", fridge);

  // Execute behavior tree at 50hz
  auto status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    // This will wake up if an Async action completes
    //tree.sleep(std::chrono::milliseconds(20));
  }

  if (status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_ERROR(node->get_logger(), "Behavior tree failed");
  }

  rclcpp::shutdown();

  return 0;
}
