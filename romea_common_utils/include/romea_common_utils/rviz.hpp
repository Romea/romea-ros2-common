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


#ifndef ROMEA_COMMON_UTILS__RVIZ_HPP_
#define ROMEA_COMMON_UTILS__RVIZ_HPP_

// ros
#include "rviz_visual_tools/rviz_visual_tools.hpp"

// romea
#include "romea_core_common/geometry/Pose2D.hpp"
#include "romea_core_common/geometry/Position2D.hpp"

namespace romea
{

void publish(
  rviz_visual_tools::RvizVisualTools & rvizVisualTool,
  const romea::Pose2D & bodyPose2D,
  const rviz_visual_tools::Colors & color,
  double positionAlongZBodyAxis = 0,
  double scaleAlongZBodyAxis = 0.1,
  double sigma = 3);

void publish(
  rviz_visual_tools::RvizVisualTools & rvizVisualTool,
  const romea::Position2D & bodyPosition2D,
  const rviz_visual_tools::Colors & color,
  double positionAlongZBodyAxis = 0,
  double scaleAlongZBodyAxis = 0.1,
  double sigma = 3);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__RVIZ_HPP_
