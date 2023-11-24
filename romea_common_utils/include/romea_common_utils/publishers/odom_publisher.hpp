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


#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__ODOM_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__ODOM_PUBLISHER_HPP_


// std
#include <string>
#include <memory>
#include <utility>

// ros
#include "nav_msgs/msg/odometry.hpp"

// romea
#include "romea_common_utils/publishers/stamped_publisher.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{
namespace ros2
{

template<typename DataType, typename NodeType>
class OdomPublisher : public StampedPublisher<DataType, nav_msgs::msg::Odometry, NodeType>
{
private:
  using Base = StampedPublisher<DataType, nav_msgs::msg::Odometry, NodeType>;
  using Options = typename Base::Options;

public:
  OdomPublisher(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name,
    const std::string & frame_id,
    const std::string & child_frame_id,
    const rclcpp::QoS & qos,
    const bool & activated);

  void publish(
    const core::Duration & duration,
    const DataType & data);

  void publish(
    const rclcpp::Time & stamp,
    const DataType & data);

public:
  std::string frame_id_;
  std::string child_frame_id_;
};


//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
OdomPublisher<DataType, NodeType>::OdomPublisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const std::string & frame_id,
  const std::string & child_frame_id,
  const rclcpp::QoS & qos,
  const bool & activated)
: Base(node, topic_name, qos, Options(), activated),
  frame_id_(frame_id),
  child_frame_id_(child_frame_id)
{
  assert(!frame_id.empty());
  assert(!child_frame_id.empty());
}

//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
void OdomPublisher<DataType, NodeType>::publish(
  const core::Duration & duration,
  const DataType & data)
{
  publish(to_ros_time(duration), data);
}

//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
void OdomPublisher<DataType, NodeType>::publish(
  const rclcpp::Time & stamp,
  const DataType & data)
{
  auto msg = std::make_unique<nav_msgs::msg::Odometry>();
  to_ros_odom_msg(stamp, data, frame_id_, child_frame_id_, *msg.get());
  this->publish_message_(std::move(msg));
}

//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
std::shared_ptr<OdomPublisher<DataType, NodeType>>
make_odom_publisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const std::string & frame_id,
  const std::string & child_frame_id,
  const rclcpp::QoS & qos,
  const bool & activated)
{
  using Publisher = OdomPublisher<DataType, NodeType>;
  return std::make_shared<Publisher>(
    node,
    topic_name,
    frame_id,
    child_frame_id,
    qos,
    activated);
}

}  // namespace ros2
}  // namespace romea


#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__ODOM_PUBLISHER_HPP_
