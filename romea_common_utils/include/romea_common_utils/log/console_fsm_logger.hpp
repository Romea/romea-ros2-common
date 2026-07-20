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

#ifndef ROMEA_COMMON_UTILS__LOG__CONSOLE_FSM_LOGGER_HPP_
#define ROMEA_COMMON_UTILS__LOG__CONSOLE_FSM_LOGGER_HPP_

// romea
#include "romea_core_common/fsm/FSMEvent.hpp"

// ros
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace romea
{
namespace ros2
{

class ConsoleFSMLogger
{
public:
  explicit ConsoleFSMLogger(rclcpp::Logger logger, bool activated = true)
  : logger_(logger), activated_(activated)
  {
  }

  void log(const core::FSMEvent & event) const
  {
    if (activated_) {
      RCLCPP_INFO(
        logger_,
        "FSM: %s -> %s: %s",
        event.previous_state.name.c_str(),
        event.current_state.name.c_str(),
        event.description.c_str());
    }
  }

  void activate() { activated_ = true; }

  void deactivate() { activated_ = false; }

  bool is_activated() const { return activated_; }

private:
  rclcpp::Logger logger_;
  bool activated_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__LOG__CONSOLE_FSM_LOGGER_HPP_
