#include "romea_common_utils/conversions/pose_and_twist2d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg)
{
  to_ros_msg(romea_pose_and_twist2d.pose, ros_pose_and_twist2d_msg.pose);
  to_ros_msg(romea_pose_and_twist2d.twist, ros_pose_and_twist2d_msg.twist);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped)
{
  ros_pose_and_twist2d_msg_stamped.header.stamp = stamp;
  ros_pose_and_twist2d_msg_stamped.header.frame_id = frame_id;
  to_ros_msg(romea_pose_and_twist2d.pose, ros_pose_and_twist2d_msg_stamped.pose);
  to_ros_msg(romea_pose_and_twist2d.twist, ros_pose_and_twist2d_msg_stamped.twist);
}

//-----------------------------------------------------------------------------
PoseAndTwist2D to_romea(const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg)
{
  PoseAndTwist2D romea_pose_and_twist2d;
  to_romea(ros_pose_and_twist2d_msg, romea_pose_and_twist2d);
  return romea_pose_and_twist2d;
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg,
  PoseAndTwist2D & romea_pose_and_twist2d)
{
  to_romea(ros_pose_and_twist2d_msg.pose, romea_pose_and_twist2d.pose);
  to_romea(ros_pose_and_twist2d_msg.twist, romea_pose_and_twist2d.twist);
}

}  // namespace romea
