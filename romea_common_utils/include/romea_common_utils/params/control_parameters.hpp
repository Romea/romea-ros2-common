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


#ifndef ROMEA_COMMON_UTILS__PARAMS__CONTROL_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS__PARAMS__CONTROL_PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// romea core
#include "romea_core_common/control/PID.hpp"

// local
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<typename Node>
inline void declare_pid_parameters(
  std::shared_ptr<Node> node,
  const std::string & ns)
{
  declare_parameter<double>(node, ns, "kp");
  declare_parameter<double>(node, ns, "ki");
  declare_parameter<double>(node, ns, "kd");
  declare_parameter<double>(node, ns, "imin");
  declare_parameter<double>(node, ns, "imax");
  declare_parameter<double>(node, ns, "error_epsilon");

}


//-----------------------------------------------------------------------------
template<typename Node>
inline PID::Parameters get_pid_parameters(
  std::shared_ptr<Node> node,
  const std::string & ns)
{
  return {
    get_parameter<double>(node, ns, "kp"),
    get_parameter<double>(node, ns, "ki"),
    get_parameter<double>(node, ns, "kd"),
    get_parameter<double>(node, ns, "imin"),
    get_parameter<double>(node, ns, "imax"),
    get_parameter<double>(node, ns, "error_epsilon")
  };
}


}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PARAMS__CONTROL_PARAMETERS_HPP_
