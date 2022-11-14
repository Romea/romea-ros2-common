#ifndef ROMEA_COMMON_UTILS_RVIZ_HPP_
#define ROMEA_COMMON_UTILS_RVIZ_HPP_

// ros
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// romea
#include <romea_core_common/geometry/Pose2D.hpp>
#include <romea_core_common/geometry/Position2D.hpp>

namespace romea
{

void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const romea::Pose2D & bodyPose2D,
             const rviz_visual_tools::Colors & color,
             double positionAlongZBodyAxis = 0,
             double scaleAlongZBodyAxis = 0.1,
             double sigma = 3);

void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const romea::Position2D & bodyPosition2D,
             const rviz_visual_tools::Colors & color,
             double positionAlongZBodyAxis = 0,
             double scaleAlongZBodyAxis = 0.1,
             double sigma = 3);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_RVIZ_HPP_
