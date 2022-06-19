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

#ifndef BT_BEER_ME_ACTION_CLIENT_HPP
#define BT_BEER_ME_ACTION_CLIENT_HPP

#include <map>
#include <memory>
#include <behaviortree_cpp_v3/action_node.h>
#include <bt_beer_me/ros2_singletons.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

template <typename ActionT>
class ActionClient : public BT::AsyncActionNode
{
protected:
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using ActionWrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using ActionGoal =  typename ActionT::Goal;
  using ActionResult = typename ActionT::Result::SharedPtr;
  using ActionType = ActionT;

  enum class State
  {
    ACTIVE,
    SUCCEEDED,
    FAILED,
  };

public:
  ActionClient(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
  {
  }

private:
  void resultCallback(const ActionWrappedResult& result)
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

  // Get a singleton for the rclcpp_action interface
  static std::shared_ptr<rclcpp_action::Client<ActionT>> getClient(const std::string& name)
  {
    static std::map<std::string, std::shared_ptr<rclcpp_action::Client<ActionT>>> client_map;
    // See if the action client exists
    auto client_it = client_map.find(name);
    if (client_it != client_map.end())
    {
      return client_it->second;
    }
    // Need to create a new action client
    auto node = getNode();
    auto client = rclcpp_action::create_client<ActionT>(node, name);
    client_map[name] = client;
    return client;
  }

protected:
  bool connect(const std::string& name)
  {
    if (!client_)
    {
      // Need to create action client
      client_ = getClient(name);
      if (!client_->wait_for_action_server(std::chrono::seconds(1)))
      {
        auto node = getNode();
        RCLCPP_ERROR(node->get_logger(), "Timed out connecting to %s", name.c_str());
        return false;
      }
    }
    return true;
  }

  void sendGoal(const ActionGoal& goal)
  {
    // Setup callbacks
    auto goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    goal_options.result_callback =
      std::bind(&ActionClient<ActionT>::resultCallback, this, std::placeholders::_1);
    goal_state_ = State::ACTIVE;
    client_->async_send_goal(goal, goal_options);
  }

  // TODO: add timeout
  BT::NodeStatus waitForResult()
  {
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

  std::shared_ptr<rclcpp_action::Client<ActionT>> client_;
  ActionResult result_;
  State goal_state_;
};

#endif  // BT_BEER_ME_ACTION_CLIENT_HPP
