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


#ifndef TEST_NODE_PUBLISHER_UTILS_HPP_
#define TEST_NODE_PUBLISHER_UTILS_HPP_

// eigen
#include <Eigen/Core>

// std
#include <memory>
#include <string>

// ros
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"


inline void to_ros_msg(
  const std::string & data,
  std_msgs::msg::String & msg)
{
  msg.data = data;
}

inline void to_ros_odom_msg(
  const rclcpp::Time & stamp,
  const nav_msgs::msg::Odometry & data,
  const std::string & frame_id,
  const std::string & child_frame_id,
  nav_msgs::msg::Odometry & msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.pose = data.pose;
  msg.twist = data.twist;
}


inline void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Eigen::Vector3d & data,
  geometry_msgs::msg::PointStamped & msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.point.x = data.x();
  msg.point.y = data.y();
  msg.point.z = data.z();
}


template<typename MsgType>
class Subscription
{
public:
  template<typename Node>
  Subscription(
    std::shared_ptr<Node> node,
    const std::string & topic_name)
  : data_()
  {
    auto callback = std::bind(&Subscription<MsgType>::cb_, this, std::placeholders::_1);
    sub_ = node->template create_subscription<MsgType>(topic_name, 0, callback);
  }

  size_t get_publisher_count()const
  {
    return sub_->get_publisher_count();
  }

  const MsgType & get_received_data() const
  {
    return data_;
  }

private:
  void cb_(typename MsgType::ConstSharedPtr msg)
  {
    data_ = *msg;
  }

  MsgType data_;
  std::shared_ptr<rclcpp::Subscription<MsgType>> sub_;
};

#endif  // TEST_NODE_PUBLISHER_UTILS_HPP_
