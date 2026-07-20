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

// local
#include "romea_common_utils/conversions/fsm_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::FSMState & romea_fsm_state,
  romea_common_msgs::msg::FSMState & ros_fsm_state_msg)
{
  ros_fsm_state_msg.name = romea_fsm_state.name;
  ros_fsm_state_msg.id = romea_fsm_state.id;
}

//-----------------------------------------------------------------------------
romea_common_msgs::msg::FSMState to_ros_msg(const core::FSMState & romea_fsm_state)
{
  romea_common_msgs::msg::FSMState ros_fsm_state_msg;
  to_ros_msg(romea_fsm_state, ros_fsm_state_msg);
  return ros_fsm_state_msg;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::FSMEvent & romea_fsm_event,
  romea_common_msgs::msg::FSMEvent & ros_fsm_event_msg)
{
  to_ros_msg(romea_fsm_event.previous_state, ros_fsm_event_msg.previous_state);
  to_ros_msg(romea_fsm_event.current_state, ros_fsm_event_msg.current_state);
  ros_fsm_event_msg.description = romea_fsm_event.description;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const core::FSMEvent & romea_fsm_event,
  romea_common_msgs::msg::FSMEvent & ros_fsm_event_msg)
{
  ros_fsm_event_msg.header.stamp = stamp;
  to_ros_msg(romea_fsm_event, ros_fsm_event_msg);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::FSMEvent & romea_fsm_event,
  romea_common_msgs::msg::FSMEvent & ros_fsm_event_msg)
{
  ros_fsm_event_msg.header.frame_id = frame_id;
  to_ros_msg(stamp, romea_fsm_event, ros_fsm_event_msg);
}

//-----------------------------------------------------------------------------
romea_common_msgs::msg::FSMEvent to_ros_msg(
  const rclcpp::Time & stamp,
  const core::FSMEvent & romea_fsm_event)
{
  romea_common_msgs::msg::FSMEvent ros_fsm_event_msg;
  to_ros_msg(stamp, romea_fsm_event, ros_fsm_event_msg);
  return ros_fsm_event_msg;
}

//-----------------------------------------------------------------------------
void to_romea(const romea_common_msgs::msg::FSMState & msg, core::FSMState & fsm_state)
{
  fsm_state.name = msg.name;
  fsm_state.id = msg.id;
}

//-----------------------------------------------------------------------------
core::FSMState to_romea(const romea_common_msgs::msg::FSMState & msg)
{
  core::FSMState fsm_state;
  to_romea(msg, fsm_state);
  return fsm_state;
}

//-----------------------------------------------------------------------------
void to_romea(const romea_common_msgs::msg::FSMEvent & msg, core::FSMEvent & fsm_event)
{
  to_romea(msg.previous_state, fsm_event.previous_state);
  to_romea(msg.current_state, fsm_event.current_state);
  fsm_event.description = msg.description;
}

//-----------------------------------------------------------------------------
core::FSMEvent to_romea(const romea_common_msgs::msg::FSMEvent & msg)
{
  core::FSMEvent fsm_event;
  to_romea(msg, fsm_event);
  return fsm_event;
}

}  // namespace ros2
}  // namespace romea
