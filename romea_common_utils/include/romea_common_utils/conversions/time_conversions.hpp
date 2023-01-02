// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__TIME_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__TIME_CONVERSIONS_HPP_

// ros
#include <rclcpp/time.hpp>
#include <message_filters/message_traits.h>

// romea core
#include <romea_core_common/time/Time.hpp>

namespace romea
{

rclcpp::Time to_ros_time(
  const romea::Duration & duration,
  rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

romea::Duration to_romea_duration(const rclcpp::Time & time);

template<typename Msg>
rclcpp::Time extract_time(const Msg & msg)
{
  return message_filters::message_traits::TimeStamp<Msg>::value(msg);
}

template<typename Msg>
Duration extract_duration(const Msg & msg)
{
  return to_romea_duration(extract_time(msg));
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__TIME_CONVERSIONS_HPP_
