#ifndef ROMEA_COMMON_UTILS_PUBLISHERS_STAMPED_DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS_PUBLISHERS_STAMPED_DATA_PUBLISHER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_common_utils/publishers/stamped_publisher.hpp"
#include "../conversions/time_conversions.hpp"

namespace romea {

template <typename DataType, typename MsgType, typename NodeType>
class StampedDataPublisher : public StampedPublisher<DataType, MsgType, NodeType>
{
private :
  using Base = StampedPublisher<DataType, MsgType, NodeType>;
  using Options = typename Base::Options;

public :
  StampedDataPublisher(std::shared_ptr<NodeType> node,
                       const std::string & topic_name,
                       const std::string & frame_id,
                       const rclcpp::QoS & qos,
                       const bool & activated);

  void publish(const rclcpp::Time & stamp,
               const DataType & data) override;


  void publish(const romea::Duration & duration,
               const DataType & data) override;

private :
  std::string frame_id_;
};

//-----------------------------------------------------------------------------
template <typename DataType, typename MsgType, typename NodeType>
StampedDataPublisher<DataType, MsgType, NodeType>::
StampedDataPublisher(std::shared_ptr<NodeType> node,
                     const std::string & topic_name,
                     const std::string & frame_id,
                     const rclcpp::QoS & qos,
                     const bool & activated):
  Base(node, topic_name, qos, Options(), activated),
  frame_id_(frame_id)
{
  assert(!frame_id.empty());
}

//-----------------------------------------------------------------------------
template <typename DataType, typename MsgType, typename NodeType>
void StampedDataPublisher<DataType, MsgType, NodeType>::
publish(const rclcpp::Time & stamp, const DataType &data)
{
  auto msg = std::make_unique<MsgType>();
  to_ros_msg(stamp, frame_id_, data, *msg.get());
  this->publish_message_(std::move(msg));
}

//-----------------------------------------------------------------------------
template <typename DataType, typename MsgType, typename NodeType>
void StampedDataPublisher<DataType, MsgType, NodeType>::
publish(const romea::Duration & duration, const DataType & data)
{
  publish(to_ros_time(duration), data);
}

//-----------------------------------------------------------------------------
template <typename DataType, typename MsgType, typename NodeType>
std::shared_ptr<StampedDataPublisher<DataType, MsgType, NodeType>>
make_stamped_data_publisher(std::shared_ptr<NodeType> node,
                            const std::string & topic_name,
                            const std::string & frame_id,
                            const rclcpp::QoS & qos,
                            const bool & activated)
{
  using Publisher = StampedDataPublisher<DataType, MsgType, NodeType>;
  return std::make_shared<Publisher>(node, topic_name, frame_id, qos, activated);
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_PUBLISHERS_STAMPED_DATA_PUBLISHER_HPP_
