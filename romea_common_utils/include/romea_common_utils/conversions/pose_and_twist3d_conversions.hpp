#ifndef _romea_OdomConversions_hpp_
#define _romea_OdomConversions_hpp_

//std
#include <string>

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/PoseAndTwist3D.hpp>
#include "pose3d_conversions.hpp"
#include "twist3d_Conversions.hpp"

//romea_ros_msg
#include <nav_msgs/msg/odometry.hpp>

namespace romea
{

void  to_ros_odom_msg(const rclcpp::Time & stamp,
                      const PoseAndTwist3D & poseAndBodyTwist3D,
                      const std::string & frame_id,
                      const std::string & child_frame_id,
                      nav_msgs::msg::Odometry & odom_msg);


}// namespace

#endif
