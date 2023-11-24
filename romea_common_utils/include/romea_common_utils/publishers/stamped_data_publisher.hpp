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


#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// local
#include "romea_common_utils/publishers/stamped_publisher.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{
namespace ros2
{

template<typename DataType, typename MsgType, typename NodeType>
class StampedDataPublisher : public StampedPublisher<DataType, MsgType, NodeType>
{
private:
  using Base = StampedPublisher<DataType, MsgType, NodeType>;
  using Options = typename Base::Options;

public:
  StampedDataPublisher(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name,
    const std::string & frame_id,
    const rclcpp::QoS & qos,
    const bool & activated);

  void publish(
    const rclcpp::Time & stamp,
    const DataType & data) override;


  void publish(
    const core::Duration & duration,
    const DataType & data) override;

private:
  std::string frame_id_;
};

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
StampedDataPublisher<DataType, MsgType, NodeType>::StampedDataPublisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const std::string & frame_id,
  const rclcpp::QoS & qos,
  const bool & activated)
: Base(node, topic_name, qos, Options(), activated),
  frame_id_(frame_id)
{
  assert(!frame_id.empty());
}

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
void StampedDataPublisher<DataType, MsgType, NodeType>::publish(
  const rclcpp::Time & stamp,
  const DataType & data)
{
  auto msg = std::make_unique<MsgType>();
  to_ros_msg(stamp, frame_id_, data, *msg.get());
  this->publish_message_(std::move(msg));
}

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
void StampedDataPublisher<DataType, MsgType, NodeType>::publish(
  const core::Duration & duration,
  const DataType & data)
{
  publish(to_ros_time(duration), data);
}

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
std::shared_ptr<StampedDataPublisher<DataType, MsgType, NodeType>>
make_stamped_data_publisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const std::string & frame_id,
  const rclcpp::QoS & qos,
  const bool & activated)
{
  using Publisher = StampedDataPublisher<DataType, MsgType, NodeType>;
  return std::make_shared<Publisher>(node, topic_name, frame_id, qos, activated);
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_
