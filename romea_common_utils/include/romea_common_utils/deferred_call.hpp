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

#ifndef ROMEA_COMMON_UTILS__DEFERRED_CALL_HPP_
#define ROMEA_COMMON_UTILS__DEFFERED_CALL_HPP_

// std
#include <utility>

// rclcpp
#include "rclcpp/timer.hpp"

namespace romea
{

class DeferredCall
{
public:
  template<typename Node, typename CallbackT>
  DeferredCall(Node & node, CallbackT && callback)
  : timer_(node.create_wall_timer({}, [this, cb = std::forward<CallbackT>(callback)] {
        timer_->cancel();
        cb();
      }))
  {
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

template<typename Node, typename CallbackT>
auto deferred_call(Node & node, CallbackT && callback)
{
  return std::make_unique<DeferredCall>(node, std::forward<CallbackT>(callback));
}

template<typename Node, typename CallbackT>
auto deferred_call(Node * node, CallbackT && callback)
{
  return std::make_unique<DeferredCall>(*node, std::forward<CallbackT>(callback));
}

template<typename Node, typename CallbackT>
auto deferred_call(std::shared_ptr<Node> node, CallbackT && callback)
{
  return std::make_unique<DeferredCall>(*node, std::forward<CallbackT>(callback));
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__DEFERRED_CALL_HPP_
