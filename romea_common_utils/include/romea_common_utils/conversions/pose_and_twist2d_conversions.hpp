#ifndef _romea_PoseAndTwist2DConversions_hpp_
#define _romea_PoseAndTwist2DConversions_hpp_

//romea
#include <romea_core_common/geometry/PoseAndTwist2D.hpp>
#include "pose2d_conversions.hpp"
#include "twist2d_conversions.hpp"

//romea_ros_msg
#include <romea_common_msgs/msg/pose_and_twist2_d.hpp>
#include <romea_common_msgs/msg/pose_and_twist2_d_stamped.hpp>

namespace romea
{


void to_ros_msg(const PoseAndTwist2D & romea_pose_and_twist2d,
              romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg);

void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const PoseAndTwist2D & romea_pose_and_twist2d,
              romea_common_msgs::msg::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped);

PoseAndTwist2D to_romea(const romea_common_msgs::msg::PoseAndTwist2D &ros_pose_and_twist2d_msg);

void to_romea(const romea_common_msgs::msg::PoseAndTwist2D &ros_pose_and_twist2d_msg,
             PoseAndTwist2D & romea_pose_and_twist2d);


}

#endif
