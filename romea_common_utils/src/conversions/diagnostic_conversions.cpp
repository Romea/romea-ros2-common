// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// local
#include "romea_common_utils/conversions/diagnostic_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_diagnostic_msg(
  const std::string & diagnoctic_name,
  const std::string & hardware_id,
  const DiagnosticReport & report,
  diagnostic_msgs::msg::DiagnosticStatus & msg)
{
  msg.level = static_cast<int>(worseStatus(report.diagnostics));
  msg.hardware_id = hardware_id;
  msg.name = diagnoctic_name;

  for (const auto & diagnostic : report.diagnostics) {
    if (diagnostic.status != DiagnosticStatus::OK) {
      msg.message += diagnostic.message + " ";
    }
  }

  for (const auto & p : report.info) {
    diagnostic_msgs::msg::KeyValue key;
    key.key = p.first;
    key.value = p.second;
    msg.values.push_back(key);
  }
}

}  // namespace romea
