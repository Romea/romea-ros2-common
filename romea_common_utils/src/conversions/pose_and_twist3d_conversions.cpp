// romea
#include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_odom_msg(
  const rclcpp::Time & stamp,
  const PoseAndTwist3D & poseAndBodyTwist3D,
  const std::string & frame_id,
  const std::string & child_frame_id,
  nav_msgs::msg::Odometry & odom_msg)
{
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = frame_id;
  odom_msg.child_frame_id = child_frame_id;
  to_ros_msg(poseAndBodyTwist3D.pose, odom_msg.pose);
  to_ros_msg(poseAndBodyTwist3D.twist, odom_msg.twist);
}

}  // namespace romea
