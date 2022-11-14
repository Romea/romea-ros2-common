//romea
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
void to_ros_msg(const Twist2D & romea_twist2d, romea_common_msgs::msg::Twist2D & ros_twist2d_msg)
{
  ros_twist2d_msg.linear_speeds.x = romea_twist2d.linearSpeeds.x();
  ros_twist2d_msg.linear_speeds.y = romea_twist2d.linearSpeeds.y();
  ros_twist2d_msg.angular_speed = romea_twist2d.angularSpeed;

  for (size_t n = 0;n < 9; ++n)
  {
    ros_twist2d_msg.covariance[n] = romea_twist2d.covariance(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const Twist2D & romea_twist_2d,
              romea_common_msgs::msg::Twist2DStamped & ros_twist2d_msg)
{
  ros_twist2d_msg.header.stamp = stamp;
  ros_twist2d_msg.header.frame_id = frame_id;
  to_ros_msg(romea_twist_2d, ros_twist2d_msg.twist);
}


//-----------------------------------------------------------------------------
void to_romea(const romea_common_msgs::msg::Twist2D &ros_twist2d_msg,
             Twist2D & romea_twist2d)
{
  romea_twist2d.linearSpeeds.x() = ros_twist2d_msg.linear_speeds.x;
  romea_twist2d.linearSpeeds.y() = ros_twist2d_msg.linear_speeds.y;
  romea_twist2d.angularSpeed = ros_twist2d_msg.angular_speed;
  romea_twist2d.covariance = Eigen::Matrix3d(ros_twist2d_msg.covariance.data());
}

//-----------------------------------------------------------------------------
Twist2D to_romea(const romea_common_msgs::msg::Twist2D & ros_twist2d_msg)
{
  Twist2D romea_twist2d;
  to_romea(ros_twist2d_msg, romea_twist2d);
  return romea_twist2d;
}


}  // namespace romea

