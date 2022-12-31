#ifndef _romea_RealtimeStampedMessagePublisher_hpp_
#define _romea_RealtimeStampedMessagePublisher_hpp_

// ros
#include <realtime_tools/realtime_publisher.h>

// std
#include <memory>
#include <string>

// romea
#include "../conversions/time_conversions.hpp"

namespace romea
{

template<class DataType, class MessageType>
class RealtimeStampedMessagePublisher
{
private:
  using Publisher = typename rclcpp::Publisher<MessageType>;
  using RTPublisher = typename realtime_tools::RealtimePublisher<MessageType>;

public:
  RealtimeStampedMessagePublisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name,
    const std::string & frame_id,
    const rclcpp::QoS & qos);

  void publish(
    const rclcpp::Time & stamp,
    const DataType & data);

  void publish(
    const romea::Duration & duration,
    const DataType & data);

protected:
  std::string frame_id_;
  std::shared_ptr<Publisher> pub_;
  std::shared_ptr<RTPublisher> rt_pub_;
};


//-----------------------------------------------------------------------------
template<class DataType, class MessageType>
RealtimeStampedMessagePublisher<DataType, MessageType>::RealtimeStampedMessagePublisher(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name,
  const std::string & frame_id,
  const rclcpp::QoS & qos)
: frame_id_(frame_id),
  pub_(node->create_publisher<MessageType>(topic_name, qos)),
  rt_pub_(std::make_shared<RTPublisher>(pub_))
{
  assert(!frame_id.empty());
}

//-----------------------------------------------------------------------------
template<class DataType, class MessageType>
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
template<class DataType, class MessageType>
void RealtimeStampedMessagePublisher<DataType, MessageType>::publish(
  const romea::Duration & duration,
  const DataType & data)
{
  publish(to_ros_time(duration), data);
}

}  // namespace romea

#endif  // _romea_RealtimeStampedMessagePublisher_hpp_
