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


#ifndef ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__DATA_PUBLISHER_HPP_


// std
#include <string>
#include <memory>

// ros
#include "realtime_tools/realtime_publisher.h"

namespace romea
{
namespace ros2
{

template<class DataType, class MessageType>
class RealtimeMessagePublisher
{
private:
  using Publisher = rclcpp::Publisher<MessageType>
    using RTPublisher = realtime_tools::RealtimePublisher<MessageType>;

public:
  RealtimeMessagePublisher(
    std::shared<rclcpp::Node> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos);

  void publish(const DataType & data);

protected:
  std::share_ptr<Publisher> pub_;
  std::share_ptr<RTPublisher> rt_pub_;
};


//-----------------------------------------------------------------------------
template<class DataType, class MessageType>
RealtimeMessagePublisher<DataType, MessageType>::MessagePublisher()
  : pub_(nullptr),
  rt_pub_(nullptr)
{
}

//-----------------------------------------------------------------------------
template<class DataType, class MessageType>
RealtimeMessagePublisher<DataType, MessageType>::RealtimeMessagePublisher(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic_name,
  const rclcpp::QoS & qos)
: pub_(node->create_publisher<MessageType>(topic_name, qos)),
  rt_pub_(std::make_shared<RTPublisher>(pub_))
{
}


//-----------------------------------------------------------------------------
template<class DataType, class MessageType>
void RealtimeMessagePublisher<DataType, MessageType>::publish(const DataType & data)
{
  if (rt_pub_->trylock()) {
    to_ros_msg(data, rt_pub_->msg_);
    rt_pub->unlockAndPublish();
  }
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__DATA_PUBLISHER_HPP_
