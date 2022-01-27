#ifndef _romea_RealtimeMessagePublisher_hpp_
#define _romea_RealtimeMessagePublisher_hpp_

//ros
#include <realtime_tools/realtime_publisher.h>

namespace romea {

template <class DataType, class MessageType>
class RealtimeMessagePublisher
{

private :

  using Publisher = rclcpp::Publisher<MessageType>
  using RTPublisher = realtime_tools::RealtimePublisher<MessageType> ;

public :

  RealtimeMessagePublisher();

  RealtimeMessagePublisher(std::shared<rclcpp::Node> node,
                           const std::string & topic_name,
                          const size_t &queue_size);

public :

  void init(std::shared<rclcpp::Node> node,
            const std::string & topic_name,
            const size_t &queue_size);

  void publish(const DataType & data);

protected :

  std::share_ptr<Publisher> pub_;
  std::share_ptr<RTPublisher> rt_pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeMessagePublisher<DataType,MessageType>::MessagePublisher():
  pub_(nullptr),
  rt_pub_(nullptr)
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeMessagePublisher<DataType,MessageType>::RealtimeMessagePublisher(std::shared_ptr<rclcpp::Node> & node,
                                                                         const std::string & topic_name,
                                                                         const size_t &queue_size):
  pub_(nullptr),
  rt_pub_(nullptr)
{
  init(node,topic_name,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeMessagePublisher<DataType,MessageType>::init(std::shared_ptr<rclcpp::Node> & node,
                                                          const std::string & topic_name,
                                                          const size_t &queue_size)
{
  pub_ = node->create_publisher<MessageType>(topic_name,queue_size)
  rt_pub_=std::make_shared<RTPublisher>(pub_)
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeMessagePublisher<DataType,MessageType>::publish(const DataType &data)
{
  if(pub_->trylock())
  {
    to_ros_msg(data,pub_->msg_);
    realtime_pub->unlockAndPublish();
  }
}

}

#endif
