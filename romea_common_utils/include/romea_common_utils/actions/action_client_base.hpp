// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_COMMON_UTILS__ACTIONS__ACTION_CLIENT_BASE_HPP_
#define ROMEA_COMMON_UTILS__ACTIONS__ACTION_CLIENT_BASE_HPP_

// std
#include <memory>
#include <string>
#include <chrono>
#include <sstream>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace romea
{

template<typename Action, typename Node>
class ActionClientBase
{
public:
  using Goal = typename Action::Goal;
  using Feedback = typename Action::Feedback;
  using Result = typename Action::Result;

  using ActionClient = typename rclcpp_action::Client<Action>;
  using SendGoalOptions = typename ActionClient::SendGoalOptions;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<Action>;
  using WrappedResult = typename GoalHandle::WrappedResult;

public:
  template<typename RepT, typename RatioT>
  ActionClientBase(
    std::shared_ptr<Node> node,
    const std::string & action_name,
    const std::chrono::duration<RepT, RatioT> & timeout,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group)
  : name_(action_name), node_(node),
    client_(nullptr),
    goal_handle_(nullptr)
  {
    client_ = rclcpp_action::create_client<Action>(
      node, action_name, callback_group);

    if (!client_->wait_for_action_server(timeout)) {
      std::stringstream msg;
      msg << "Failed to call ";
      msg << action_name;
      msg << " server";
      throw(std::runtime_error(msg.str()));
    }
  }

protected:
  bool server_is_always_ready()
  {
    if (!client_->action_server_is_ready()) {
      std::stringstream msg;
      msg << "Failed to send goal to ";
      msg << name_;
      msg << " action server because is not ready anymore";
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());
      return false;
    } else {
      return true;
    }
  }

  bool goal_is_accepted_(std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle)
  {
    if (future_goal_handle.get() == nullptr) {
      std::stringstream msg;
      msg << "Goal has been rejected by ";
      msg << name_;
      msg << " action server";
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());
      return false;
    } else {
      this->goal_handle_ = future_goal_handle.get();
      return true;
    }
  }

  typename Result::SharedPtr get_result_(const WrappedResult & wrapped_result)
  {
    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return wrapped_result.result;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        {
          std::stringstream msg;
          msg << "Goal has been aborted by ";
          msg << name_;
          msg << " action server";
          RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());
          return nullptr;
        }
        break;
      case rclcpp_action::ResultCode::CANCELED:
        {
          std::stringstream msg;
          msg << "Goal has been canceled by ";
          msg << name_;
          msg << " action server";
          RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());
          return nullptr;
        }
        break;
      default:
        {
          std::stringstream msg;
          msg << "Result has been sent with an unknown status by ";
          msg << name_;
          msg << " action server ";
          RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());
          return nullptr;
        }
        break;
    }
  }

protected:
  std::string name_;
  std::shared_ptr<Node> node_;
  std::shared_ptr<ActionClient> client_;
  std::shared_ptr<GoalHandle> goal_handle_;
  SendGoalOptions send_goal_options_;
};

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__ACTIONS__ACTION_CLIENT_BASE_HPP_
