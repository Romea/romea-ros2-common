#ifndef ROMEA_COMMON_UTILS_CONVERSIONS_POSE2D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS_CONVERSIONS_POSE2D_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/Pose2D.hpp>

// romea_ros_msg
#include <romea_common_msgs/msg/pose2_d_stamped.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>

// ros
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace romea
{

void to_ros_msg(const Pose2D & romea_pose2d,
              romea_common_msgs::msg::Pose2D & ros_pose2d_msg);

void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const Pose2D & romea_pose2d,
              romea_common_msgs::msg::Pose2DStamped & ros_pose2d_msg);

void to_ros_transform_msg(const Pose2D & romea_pose2d,
                       geometry_msgs::msg::Transform & ros_transform_msg);


void to_ros_transform_msg(const rclcpp::Time & stamp,
                       const Pose2D &romea_pose_2d,
                       const std::string &frame_id,
                       const std::string &child_frame_id,
                       geometry_msgs::msg::TransformStamped &ros_transform_msg);

void to_romea(const romea_common_msgs::msg::Pose2D &msg,
             Pose2D & romea_pose_2d);

Pose2D to_romea(const romea_common_msgs::msg::Pose2D &msg);

//void to_romea(const romea_localisation_msgs::Pose2DStamped &msg,
//             Pose2D::Stamped & romea_pose_2d_stamped,
//             std::string & frame_id);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_CONVERSIONS_POSE2D_CONVERSIONS_HPP_
