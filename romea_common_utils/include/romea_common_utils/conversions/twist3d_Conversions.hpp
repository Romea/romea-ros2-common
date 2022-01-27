#ifndef _romea_Twist3DConversions_hpp_
#define _romea_Twist3DConversions_hpp_


//romea
#include <romea_core_common/geometry/Twist3D.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>

//ros
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>


namespace romea
{

void to_ros_msg(const Twist3D & romea_twist_3d,
              geometry_msgs::msg::TwistWithCovariance & ros_twist_msg);

void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const Twist3D & romea_twist_3d,
              geometry_msgs::msg::TwistWithCovarianceStamped & ros_twist3d_msg);

void to_romea(const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg,
             Twist3D & romea_twist3d);

Twist3D to_romea(const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg);


}// namespace

#endif
