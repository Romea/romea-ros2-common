#ifndef _romea_Pose3DConversions_hpp_
#define _romea_Pose3DConversions_hpp_

//std
#include <string>

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/Pose3D.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>


//ros
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace romea
{


void to_ros_transform_msg(const Pose3D & romea_pose_3d,
                          geometry_msgs::msg::Transform & ros_transform_msg);


void to_ros_transform_msg(const rclcpp::Time & stamp,
                          const Pose3D &romea_pose_3d,
                          const std::string &frame_id,
                          const std::string &child_frame_id,
                          geometry_msgs::msg::TransformStamped &tf_msg);

void to_ros_msg(const Pose3D & romea_pose_3d,
                geometry_msgs::msg::PoseWithCovariance & ros_pose_msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string &frame_id,
                const Pose3D & romea_pose_3d,
                geometry_msgs::msg::PoseWithCovarianceStamped & ros_pose_msg);


void to_romea(const geometry_msgs::msg::PoseWithCovariance & ros_pose_msg,
              Pose3D & romea_pose_3d);

Pose3D to_romea(const geometry_msgs::msg::PoseWithCovariance & ros_pose_msg);

}

#endif
