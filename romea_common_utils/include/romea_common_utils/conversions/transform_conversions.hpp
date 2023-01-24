// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__TRANSFORM_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__TRANSFORM_CONVERSIONS_HPP_

// eigen
#include <Eigen/Geometry>

// std
#include <memory>
#include <string>

// ros
#include "geometry_msgs/msg/transform.hpp"
#include "rclcpp/node.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"


namespace romea
{

void to_romea(
  const geometry_msgs::msg::Transform & tranform_msg,
  Eigen::Affine3d & eigen_transform);

void to_ros_transform_msg(
  const Eigen::Affine3d & eigen_transform,
  geometry_msgs::msg::Transform & tranform_msg);

Eigen::Affine3d lookupTransformOnce(
  std::shared_ptr<rclcpp::Node> node,
  tf2_ros::Buffer & tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & time,
  const rclcpp::Duration timeout);


}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__TRANSFORM_CONVERSIONS_HPP_
