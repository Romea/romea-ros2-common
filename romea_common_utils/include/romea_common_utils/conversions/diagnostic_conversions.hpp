// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__DIAGNOSTIC_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__DIAGNOSTIC_CONVERSIONS_HPP_

// ros
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

// romea core
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

// std
#include <string>


namespace romea
{

void to_ros_diagnostic_msg(
  const std::string & diagnostic_name,
  const std::string & hardware_id,
  const DiagnosticReport & report,
  diagnostic_msgs::msg::DiagnosticStatus & msg);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__DIAGNOSTIC_CONVERSIONS_HPP_
