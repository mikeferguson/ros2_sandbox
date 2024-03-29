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

#ifndef BT_BEER_ME_ROS2_SINGLETONS_HPP
#define BT_BEER_ME_ROS2_SINGLETONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

rclcpp::Node::SharedPtr getNode()
{
  static rclcpp::Node::SharedPtr node;

  if (!node)
  {
    node = rclcpp::Node::make_shared("bt_beer_me");
  }

  return node;
}

// TODO: tf2_buffer singleton

#endif  // BT_BEER_ME_ROS2_SINGLETONS_HPP
