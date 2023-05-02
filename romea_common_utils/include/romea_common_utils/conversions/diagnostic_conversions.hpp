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


#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__DIAGNOSTIC_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__DIAGNOSTIC_CONVERSIONS_HPP_


// std
#include <string>

// ros
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

// romea core
#include "romea_core_common/diagnostic/DiagnosticReport.hpp"


namespace romea
{

void to_ros_diagnostic_msg(
  const std::string & diagnostic_name,
  const std::string & hardware_id,
  const DiagnosticReport & report,
  diagnostic_msgs::msg::DiagnosticStatus & msg);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__DIAGNOSTIC_CONVERSIONS_HPP_
