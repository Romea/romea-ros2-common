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


#ifndef ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_


// std
#include <memory>
#include <string>

// ros
#include "realtime_tools/realtime_publisher.h"

// local
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{
namespace ros2
{

template<typename DataType, typename MessageType>
class RealtimeStampedMessagePublisher
{
private:
  using Publisher = typename rclcpp::Publisher<MessageType>;
  using RTPublisher = typename realtime_tools::RealtimePublisher<MessageType>;

public:
  template<typename NodeType>
  RealtimeStampedMessagePublisher(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name,
    const std::string & frame_id,
    const rclcpp::QoS & qos);

  void publish(
    const rclcpp::Time & stamp,
    const DataType & data);

  void publish(
    const core::Duration & duration,
    const DataType & data);

protected:
  std::string frame_id_;
  std::shared_ptr<Publisher> pub_;
  std::shared_ptr<RTPublisher> rt_pub_;
};


//-----------------------------------------------------------------------------
template<typename DataType, typename MessageType>
template<typename NodeType>
RealtimeStampedMessagePublisher<DataType, MessageType>::RealtimeStampedMessagePublisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const std::string & frame_id,
  const rclcpp::QoS & qos)
: frame_id_(frame_id),
  pub_(node->template create_publisher<MessageType>(topic_name, qos)),
  rt_pub_(std::make_shared<RTPublisher>(pub_))
{
  assert(!frame_id.empty());
}

//-----------------------------------------------------------------------------
template<typename DataType, typename MessageType>
void RealtimeStampedMessagePublisher<DataType, MessageType>::publish(
  const rclcpp::Time & stamp,
  const DataType & data)
{
  if (rt_pub_->trylock()) {
    to_ros_msg(stamp, frame_id_, data, rt_pub_->msg_);
    rt_pub_->unlockAndPublish();
  }
}

//-----------------------------------------------------------------------------
template<typename DataType, typename MessageType>
void RealtimeStampedMessagePublisher<DataType, MessageType>::publish(
  const core::Duration & duration,
  const DataType & data)
{
  publish(to_ros_time(duration), data);
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_
