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

// std
#include <memory>
#include <string>

// ros
#include "romea_common_utils/ros_versions.hpp"
#if ROS_DISTRO == ROS_GALACTIC
#include "tf2_eigen/tf2_eigen.h"
#else
#include "tf2_eigen/tf2_eigen.hpp"
#endif
#include "tf2_ros/transform_listener.h"


// local
#include "romea_common_utils/conversions/transform_conversions.hpp"
#include "romea_common_utils/conversions/geometry_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::Transform & tranform_msg,
  Eigen::Affine3d & eigen_transform)
{
  eigen_transform.translation() = Eigen::Vector3d(
    tranform_msg.translation.x,
    tranform_msg.translation.y,
    tranform_msg.translation.z);

  Eigen::Quaterniond q(tranform_msg.rotation.w,
    tranform_msg.rotation.x,
    tranform_msg.rotation.y,
    tranform_msg.rotation.z);

  eigen_transform.linear() = q.toRotationMatrix();
}

//-----------------------------------------------------------------------------
void to_ros_transform_msg(
  const Eigen::Affine3d & eigen_transform,
  geometry_msgs::msg::Transform & tranform_msg)
{
  to_ros_msg(eigen_transform.translation(), tranform_msg.translation);
  to_ros_msg(eigen_transform.linear(), tranform_msg.rotation);
}

// //-----------------------------------------------------------------------------
// Eigen::Affine3d lookupTransformOnce(
//   std::shared_ptr<rclcpp::Node> node,
//   tf2_ros::Buffer & tf_buffer,
//   const std::string & target_frame,
//   const std::string & source_frame,
//   const rclcpp::Time & time,
//   const rclcpp::Duration timeout)
// {
// //  tf2_ros::Buffer tf_buffer(node->get_clock());
//   tf2_ros::TransformListener tf_listener(tf_buffer);

//   auto tranform_msg = tf_buffer.lookupTransform(
//     target_frame,
//     source_frame,
//     time,
//     timeout);

//   return tf2::transformToEigen(tranform_msg);
// }

}  // namespace ros2
}  // namespace romea
