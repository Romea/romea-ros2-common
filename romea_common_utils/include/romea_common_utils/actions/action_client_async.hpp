// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__ACTIONS__ACTION_CLIENT_ASYNC_HPP_
#define ROMEA_COMMON_UTILS__ACTIONS__ACTION_CLIENT_ASYNC_HPP_

// std
#include <memory>
#include <string>
#include <chrono>
#include <sstream>

#include "romea_common_utils/actions/action_client_base.hpp"

namespace romea
{

template<typename Action, typename Node>
class ActionClientAsync : public ActionClientBase<Action, Node>
{
public:
  using Goal = typename Action::Goal;
  using Feedback = typename Action::Feedback;
  using Result = typename Action::Result;

  using GoalHandle = typename rclcpp_action::ClientGoalHandle<Action>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using WrappedResult = typename GoalHandle::WrappedResult;

public:
  template<typename RepT, typename RatioT>
  ActionClientAsync(
    std::shared_ptr<Node> node,
    const std::string & action_name,
    const std::chrono::duration<RepT, RatioT> & timeout,
    FeedbackCallback feedback_callback = nullptr,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group = nullptr)
  : ActionClientBase<Action, Node>(node, action_name, timeout, callback_group)
  {
    this->send_goal_options_.feedback_callback = feedback_callback;
  }

  template<typename RepT, typename RatioT>
  std::shared_ptr<Result> send_goal(
    Goal goal,
    std::chrono::duration<RepT, RatioT> timeout)
  {
    if (!this->server_is_always_ready()) {
      return nullptr;
    }

    auto future_goal_handle = this->client_->async_send_goal(goal, this->send_goal_options_);

    if (!this->wait_goal_handle(future_goal_handle, timeout) ||
      !this->goal_is_accepted_(future_goal_handle))
    {
      return nullptr;
    }

    auto future_wrapped_result = this->client_->async_get_result(future_goal_handle.get());
    if (this->wait_result_(future_wrapped_result, timeout)) {
      return this->get_result_(future_wrapped_result.get());
    } else {
      return nullptr;
    }
  }

  void cancel_goal()
  {
    // using namespace std::chrono_literals;
    auto future_cancel_status = this->client_->async_cancel_goal(this->goal_handle_);
    auto future_status = future_cancel_status.wait_for(std::chrono::milliseconds(10));
    assert(future_status != std::future_status::deferred);

    if (future_status != std::future_status::ready || !future_cancel_status.get()) {
      std::stringstream msg;
      msg << "Failed to cancel goal from ";
      msg << this->name_;
      msg << " action server ";
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());
    }
  }

  template<typename RepT, typename RatioT>
  bool wait_goal_handle(
    std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle,
    std::chrono::duration<RepT, RatioT> timeout)
  {
    auto future_status = future_goal_handle.wait_for(timeout);
    assert(future_status != std::future_status::deferred);

    if (future_status == std::future_status::timeout) {
      std::stringstream msg;
      msg << "Goal is canceled by ";
      msg << this->name_;
      msg << " action server because a timeout occured when waiting goal handle ";
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());

      cancel_goal();
      return false;
    } else {
      return true;
    }
  }

  template<typename RepT, typename RatioT>
  bool wait_result_(
    std::shared_future<WrappedResult> future_wrapped_result,
    std::chrono::duration<RepT, RatioT> timeout)
  {
    auto future_status = future_wrapped_result.wait_for(timeout);
    assert(future_status != std::future_status::deferred);

    if (future_status == std::future_status::timeout) {
      std::stringstream msg;
      msg << "Goal is canceled by ";
      msg << this->name_;
      msg << " action server because a timeout occured when waiting result ";
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), msg.str());

      cancel_goal();
      return false;
    } else {
      return true;
    }
  }
};


}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__ACTIONS__ACTION_CLIENT_ASYNC_HPP_
