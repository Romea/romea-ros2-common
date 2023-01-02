// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__DATA_PUBLISHER_HPP_

// ros
#include <realtime_tools/realtime_publisher.h>

// std
#include <string>
#include <memory>

namespace romea
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

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__REALTIME_PUBLISHERS__DATA_PUBLISHER_HPP_
