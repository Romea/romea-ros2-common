#ifndef _romea_OdomPublisher_hpp_
#define _romea_OdomPublisher_hpp_

//ros
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <nav_msgs/msg/odometry.hpp>

//romea
#include "../conversions/time_conversions.hpp"
#include "../qos.hpp"

namespace romea
{

template <class DataType>
class OdomPublisher
{

public :

  OdomPublisher(std::shared_ptr<rclcpp::Node> node,
                const std::string & topic_name,
                const std::string & frame_id,
                const std::string & child_frame_id,
                const rclcpp::QoS & qos);

  void publish(const Duration & duration,
               const DataType &data);

  void publish(const rclcpp::Time & stamp,
               const DataType &data);

public :

  std::string frame_id_;
  std::string child_frame_id_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_;
};


//-----------------------------------------------------------------------------
template <class DataType>
OdomPublisher<DataType>::OdomPublisher(std::shared_ptr<rclcpp::Node> node,
                                       const std::string & topic_name,
                                       const std::string & frame_id,
                                       const std::string & child_frame_id,
                                       const rclcpp::QoS & qos):
  frame_id_(frame_id),
  child_frame_id_(child_frame_id),
  pub_(node->create_publisher<nav_msgs::msg::Odometry>(topic_name,qos))
{
  assert(!frame_id.empty());
  assert(!child_frame_id.empty());
}

//-----------------------------------------------------------------------------
template <class DataType>
void OdomPublisher<DataType>::publish(const Duration & duration,
                                      const DataType &data)
{
  publish(to_ros_time(duration),data);
}

//-----------------------------------------------------------------------------
template <class DataType>
void OdomPublisher<DataType>::publish(const rclcpp::Time & stamp,
                                      const DataType &data)
{
  auto msg = std::make_unique<nav_msgs::msg::Odometry>();
  to_ros_odom_msg(stamp,data,frame_id_,child_frame_id_,*msg.get());
  pub_->publish(std::move(msg));
}

}


#endif
