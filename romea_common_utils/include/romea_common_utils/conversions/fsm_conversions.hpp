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

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__FSM_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__FSM_CONVERSIONS_HPP_

// ros
#include "rclcpp/time.hpp"

// romea
#include "romea_core_common/fsm/FSMEvent.hpp"
#include "romea_core_common/fsm/FSMState.hpp"

// romea ros
#include "romea_common_msgs/msg/fsm_event.hpp"
#include "romea_common_msgs/msg/fsm_state.hpp"

namespace romea
{
namespace ros2
{

void to_ros_msg(
  const core::FSMState & romea_fsm_state,
  romea_common_msgs::msg::FSMState & ros_fsm_state_msg);

romea_common_msgs::msg::FSMState to_ros_msg(const core::FSMState & romea_fsm_state);

void to_ros_msg(
  const core::FSMEvent & romea_fsm_event,
  romea_common_msgs::msg::FSMEvent & ros_fsm_event_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const core::FSMEvent & romea_fsm_event,
  romea_common_msgs::msg::FSMEvent & ros_fsm_event_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::FSMEvent & romea_fsm_event,
  romea_common_msgs::msg::FSMEvent & ros_fsm_event_msg);

romea_common_msgs::msg::FSMEvent to_ros_msg(
  const rclcpp::Time & stamp,
  const core::FSMEvent & romea_fsm_event);

void to_romea(const romea_common_msgs::msg::FSMState & msg, core::FSMState & fsm_state);

core::FSMState to_romea(const romea_common_msgs::msg::FSMState & msg);

void to_romea(const romea_common_msgs::msg::FSMEvent & msg, core::FSMEvent & fsm_event);

core::FSMEvent to_romea(const romea_common_msgs::msg::FSMEvent & msg);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__FSM_CONVERSIONS_HPP_
