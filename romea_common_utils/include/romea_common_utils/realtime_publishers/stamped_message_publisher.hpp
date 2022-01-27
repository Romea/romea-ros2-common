#ifndef _romea_RealtimeStampedMessagePublisher_hpp_
#define _romea_RealtimeStampedMessagePublisher_hpp_

//ros
#include "../conversions/time_conversions.hpp"
#include <realtime_tools/realtime_publisher.h>

namespace romea {

template <class DataType, class MessageType>
class RealtimeStampedMessagePublisher
{

private :

  using Publisher =  typename rclcpp::Publisher<MessageType> ;
  using RTPublisher =  typename realtime_tools::RealtimePublisher<MessageType> ;

public :

  RealtimeStampedMessagePublisher();

  RealtimeStampedMessagePublisher(std::shared_ptr<rclcpp::Node> node,
                                  const std::string & topic_name,
                                  const std::string & frame_id,
                                  const size_t &queue_size);
public :

  void init(std::shared_ptr<rclcpp::Node> node,
            const std::string & topic_name,
            const std::string & frame_id,
            const size_t &queue_size);

  void publish(const rclcpp::Time & stamp,
               const DataType & data);


  void publish(const romea::Duration & duration,
               const DataType & data);

protected :

  std::string frame_id_;
  std::shared_ptr<Publisher> pub_;
  std::shared_ptr<RTPublisher> rt_pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeStampedMessagePublisher<DataType,MessageType>::RealtimeStampedMessagePublisher():
  frame_id_(),
  pub_(nullptr),
  rt_pub_(nullptr)
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeStampedMessagePublisher<DataType,MessageType>::RealtimeStampedMessagePublisher(std::shared_ptr<rclcpp::Node> node,
                                                                                       const std::string & topic_name,
                                                                                       const std::string & frame_id,
                                                                                       const size_t &queue_size):
  frame_id_(),
  pub_(nullptr),
  rt_pub_(nullptr)
{
  init(node,topic_name,frame_id,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeStampedMessagePublisher<DataType,MessageType>::init(std::shared_ptr<rclcpp::Node> node,
                                                                 const std::string & topic_name,
                                                                 const std::string & frame_id,
                                                                 const size_t &queue_size)
{
  assert(!frame_id.empty());
  frame_id_ = frame_id;
  pub_ = node->create_publisher<MessageType>(topic_name,queue_size);
  rt_pub_= std::make_shared<RTPublisher>(pub_);
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeStampedMessagePublisher<DataType,MessageType>::publish(const rclcpp::Time & stamp,
                                                                    const DataType &data)
{
  if(pub_->trylock())
  {
      to_ros_msg(stamp,frame_id_,data,pub_->msg_);
      pub_->unlockAndPublish();
  }
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeStampedMessagePublisher<DataType,MessageType>::publish(const romea::Duration & duration,
                                                                    const DataType & data)
{
  publish(to_ros_time(duration),data);
}

}

#endif
