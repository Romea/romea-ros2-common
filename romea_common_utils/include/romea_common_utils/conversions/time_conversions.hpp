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

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__TIME_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__TIME_CONVERSIONS_HPP_

// ros
#include <message_filters/message_traits.h>
#include <rclcpp/time.hpp>

// romea core
#include <romea_core_common/time/Time.hpp>

namespace romea::ros2
{

rclcpp::Time to_ros_time(
  const core::Duration & duration, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

core::Duration to_romea_duration(const rclcpp::Time & time);

core::TimePoint to_romea_time(const rclcpp::Time & time);

template<typename Msg>
rclcpp::Time extract_time(const Msg & msg)
{
  return message_filters::message_traits::TimeStamp<Msg>::value(msg);
}

template<typename Msg>
core::Duration extract_duration(const Msg & msg)
{
  return to_romea_duration(extract_time(msg));
}

}  // namespace romea::ros2

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__TIME_CONVERSIONS_HPP_
