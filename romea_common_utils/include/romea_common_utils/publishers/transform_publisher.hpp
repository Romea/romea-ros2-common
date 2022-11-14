#ifndef ROMEA_COMMON_UTILS_PUBLISHERS_TRANSFORM_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS_PUBLISHERS_TRANSFORM_PUBLISHER_HPP_

// std
#include <string>
#include <memory>

// ros
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>

// romea
#include "romea_common_utils/publishers/stamped_publisher.hpp"
#include "../conversions/time_conversions.hpp"
#include "../conversions/transform_conversions.hpp"

namespace romea {

template <typename DataType, typename NodeType>
class TransformPublisher : public StampedPublisher<DataType, tf2_msgs::msg::TFMessage, NodeType>
{
private :

  using Base = StampedPublisher<DataType, tf2_msgs::msg::TFMessage, NodeType>;
  using Options =  typename Base::Options;

public :

  TransformPublisher(std::shared_ptr<NodeType> node,
                     const std::string & frame_id,
                     const std::string & child_frame_id,
                     const bool & activated);

  virtual ~TransformPublisher() = default;

  virtual void publish(const rclcpp::Time & stamp,
                       const DataType & data);

  virtual void publish(const Duration & duration,
                       const DataType & data);

private :

  static Options make_options();

private :

  tf2_msgs::msg::TFMessage message_;
};

//-----------------------------------------------------------------------------
template <typename DataType, typename NodeType>
TransformPublisher<DataType, NodeType>::TransformPublisher(std::shared_ptr<NodeType> node,
                                                          const std::string & frame_id,
                                                          const std::string & child_frame_id,
                                                          const bool & activated):
  Base(node, "/tf", tf2_ros::DynamicBroadcasterQoS(), make_options(), activated),
  message_()
{
  assert(!frame_id.empty());
  assert(!child_frame_id.empty());

  message_.transforms.resize(1);
  message_.transforms[0].header.frame_id = frame_id;
  message_.transforms[0].child_frame_id = child_frame_id;

  message_.transforms[0].transform.translation.x = 0;
  message_.transforms[0].transform.translation.y = 0;
  message_.transforms[0].transform.translation.z = 0;
  message_.transforms[0].transform.rotation.x = 0;
  message_.transforms[0].transform.rotation.y = 0;
  message_.transforms[0].transform.rotation.z = 0;
  message_.transforms[0].transform.rotation.z = 1;
}

//-----------------------------------------------------------------------------
template <typename DataType, typename NodeType>
typename TransformPublisher<DataType, NodeType>::Options
TransformPublisher<DataType, NodeType>::make_options()
{
  Options options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
  return options;
}

//-----------------------------------------------------------------------------
template <typename DataType, typename NodeType>
void TransformPublisher<DataType, NodeType>::publish(const rclcpp::Time & stamp,
                                                    const DataType &data)
{
  message_.transforms[0].header.stamp = stamp;
  to_ros_transform_msg(data, message_.transforms[0].transform);
  this->publish_message_(message_);
}

//-----------------------------------------------------------------------------
template <typename DataType, typename NodeType>
void TransformPublisher<DataType, NodeType>::publish(const romea::Duration & duration,
                                                     const DataType & data)
{
  publish(to_ros_time(duration), data);
}

//-----------------------------------------------------------------------------
template <typename DataType, typename NodeType>
std::shared_ptr<TransformPublisher<DataType, NodeType>>
make_transform_publisher(std::shared_ptr<NodeType> node,
                         const std::string & frame_id,
                         const std::string & child_frame_id,
                         const bool & activated)
{
  using Publisher = TransformPublisher<DataType, NodeType>;
  return std::make_shared<Publisher>(node,
                                     frame_id,
                                     child_frame_id,
                                     activated);
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_PUBLISHERS_STAMPED_PUBLISHER_HPP_
